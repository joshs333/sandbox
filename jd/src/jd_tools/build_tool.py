def get_build_tool(name):
    tools = {
        "cmake": CMakeBuildTool,
        "colcon": ColconBuildTool
    }
    if name not in tools:
        raise Exception("Unknown build tool: {}".format(name))
    return tools[name]

class BuildTool():
    @staticmethod
    def getCommands():
        return [k for k in self.commands.keys()]
    
    def executeCommand(self, command, args):
        if command in self.commands:
            return self.commands[command](args)
        raise Exception("BuildEnv [{}] from [{}] does not have command [{}]".format(type(self), self.root_dir, command))

    def __init__(self, config):
        self.commands = {}

    def getBuildCommand(self, args):
        raise Exception("getBuildCommand() not implemented in {}".format(type(self)))

class CMakeBuildTool(BuildTool):
    def getBuildCommand(self, args):
        return "mkdir -p build/ && cd build/ && cmake .. && make {}".format(args)

class ColconBuildTool(BuildTool):
    def getBuildCommand(self, args):
        return "colcon build {}".format(args)