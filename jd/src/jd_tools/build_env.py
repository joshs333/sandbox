import os
import yaml
import docker

def get_build_env(name):
    envs = {
        "docker": DockerBuildEnv,
        "docker_compose": DockerComposeBuildEnv,
        "host": HostBuildEnv
    }
    if name not in envs:
        raise Exception("Unknown build tool: {}".format(name))
    return envs[name]

class SETUP():
    READY = 1
    COMMAND = 2
    MESSAGE = 3
    SETUP = 4
    FAIL = 5

class BuildEnv():
    def getCommands(self):
        return [k for k in self.commands.keys()]
    
    def getCommand(self, command, args):
        if command in self.commands:
            return self.commands[command](args)
        raise Exception("BuildEnv [{}] from [{} -> {}] does not have command [{}]".format(type(self), self.source, self.name, command))

    def __init__(self, config):
        self.name = config["name"]
        self.root_dir = config["workspace_root"]
        self.source = config["workspace_source"]
        self.source_root_dir = os.path.split(config["workspace_source"])[0]
        self.commands = {}

    def setup(self, args = None):
        return [ (SETUP.READY, "") ]
    
    def renderCommand(self, command):
        return command

class HostBuildEnv(BuildEnv):
    def __init__(self, config):
        BuildEnv.__init__(self, config)

class DockerBuildEnv(BuildEnv):
    def __init__(self, config):
        BuildEnv.__init__(self, config)
        # Defaults
        self.dockerfile = "Dockerfile"
        self.docker_tag = "jd_ws_{}".format(self.name)
        self.docker_run_args = ""
        self.docker_ws_args = "-v $(pwd):/workspace -w /workspace"
        # Override from config
        if "dockerfile" in config:
            self.dockerfile = config["dockerfile"]
        if "docker_tag" in config:
            self.docker_tag = config["docker_tag"]
        if "docker_run_args" in config:
            self.docker_run_args = config["docker_run_args"]
        if "docker_ws_args" in config:
            self.docker_ws_args = config["docker_ws_args"]
    
        self.commands = {
            "build": self.build,
            "join": self.join
        }
    
    def setup(self, args = None):
        client = docker.from_env()
        ils = [ image.tags for image in client.images.list() ]
        env_tag = self.docker_tag
        if ":" not in self.docker_tag:
            env_tag = self.docker_tag + ":latest"
        found = False
        for il in ils:
            if env_tag in il:
                found = True
                break
        
        if found:
            return [ (SETUP.READY, "") ]
        else:
            return [ (SETUP.FAIL, "Build the docker container using the build command (jd build_env build {}).".format(self.name))]

    def build(self, args):
        return 'docker build -f {} -t {} .'.format(self.dockerfile, self.docker_tag)

    def join(self, args):
        return "docker run --rm -it {} {} {}".format(self.docker_run_args, self.docker_ws_args, self.docker_tag)

    def renderCommand(self, command):
        return 'docker run --rm {} {} {} /bin/bash -c "{}"'.format(self.docker_run_args, self.docker_ws_args, self.docker_tag, command)

class DockerComposeBuildEnv(BuildEnv):
    def getImageTag(self):
        for file in self.docker_compose_files:
            if file[0] != "/":
                file = os.path.join(self.source_root_dir, file)
            try:
                with open(file, 'r') as fstream:
                    y = yaml.safe_load(fstream)
                    if "services" in y:
                        for k in y["services"]:
                            if k == self.docker_compose_service:
                                if "image" not in y["services"][k]:
                                    raise Exception("Unable to find image tag in docker compose {} service from {}.".format(k, file))
                                return y["services"][k]["image"]
            except IOError as err:
                print("Warning: error reading {}: {}".format(file, str(err)))
        raise Exception("Unable to find {} service from workspace {}".format(self.docker_compose_service, self.name))

    def getContainerName(self):
        for file in self.docker_compose_files:
            if file[0] != "/":
                file = os.path.join(self.source_root_dir, file)
            try:
                with open(file, 'r') as fstream:
                    y = yaml.safe_load(fstream)
                    if "services" in y:
                        for k in y["services"]:
                            if k == self.docker_compose_service:
                                if "container_name" not in y["services"][k]:
                                    raise Exception("Unable to find container_name tag in docker compose {} service from {}.".format(k, file))
                                return y["services"][k]["container_name"]
            except IOError as err:
                print("Warning: error reading {}: {}".format(file, str(err)))
        raise Exception("Unable to find {} service from workspace {}".format(self.docker_compose_service, self.name))

    def __init__(self, config):
        BuildEnv.__init__(self, config)
        self.docker_compose_files = ["docker-compose.yml"]
        self.docker_compose_service = "jd_ws_{}".format(self.name)
        self.docker_compose_files
        if "docker_compose_files" in config:
            self.docker_compose_files = config["docker_compose_files"]
        if "docker_compose_service" in config:
            self.docker_compose_service = config["docker_compose_service"]

        self.commands = {
            "list": self.list_services,
            "build": self.build,
            "up": self.up,
            "join": self.join,
            "stop": self.stop
        }

    def list_services(self, args = None):
        result = []
        for file in self.docker_compose_files:
            if file[0] != "/":
                file = os.path.join(self.source_root_dir, file)
            try:
                with open(file, 'r') as fstream:
                    y = yaml.safe_load(fstream)
                    if "services" in y:
                        for k in y["services"]:
                            result.append(k)
            except Exception as err:
                print("Warning: unable to process file [%s] due to [%s]"%(file, str(err)))
        if args is not None:
            print("Services available from {}:".format(self.name))
            for r in result:
                print("- {}".format(r))
            return None
        return result

    def base_command(self):
        s = self.list_services()
        if self.docker_compose_service not in s:
            raise Exception("service named: %s does not exist in provided files."%self.docker_compose_service)

        command = "docker-compose"
        for f in self.docker_compose_files:
            command += " -f " + f
        command += " -p {}".format(self.docker_compose_service)
        return command

    def setup(self, args = None):
        client = docker.from_env()
        ils = [ image.tags for image in client.images.list() ]
        env_tag = self.getImageTag()
        if ":" not in env_tag:
            env_tag = env_tag + ":latest"
        found = False
        for il in ils:
            if env_tag in il:
                found = True
                break
        if not found:
            return [
                (SETUP.FAIL, "no image named {}, run `jd build_env build {}` command to build the docker image".format(env_tag, self.name))
            ]

        client = docker.from_env()
        containers = [ c.name for c in client.containers.list() ]
        if self.getContainerName() not in containers:
            if args is None:
                return [
                    (SETUP.MESSAGE, "container named {} not running".format(self.getContainerName())),
                    (SETUP.COMMAND, self.up("")),
                    (SETUP.SETUP, 1)
                ]
            else:
                return [
                    (SETUP.FAIL, "container is still not running")
                ]

        return [
            (SETUP.READY, "")
        ]

    def stop(self, args):
        return "docker stop {}".format(self.getContainerName())

    def up(self, args):
        return self.base_command() + " up -d --force-recreate " + self.docker_compose_service + " " + args
    
    def build(self, args):
        return self.base_command() + " build " + self.docker_compose_service + " " + args

    def join(self, args = ""):
        return 'docker exec -it {} /bin/bash'.format(self.getContainerName())

    def renderCommand(self, command):
        return 'docker exec {} /bin/bash -c "source ~/.bashrc && {}"'.format(self.getContainerName(), command)