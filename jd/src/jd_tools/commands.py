import jd_tools.build_env as jmbe
import jd_tools.build_tool as jmbt
import os
import shlex
import subprocess

def workspace_build_env(workspace_config, command, args, dry_run=False, list_options=False):
    # check config for vars
    if "build_env" not in workspace_config:
        raise Exception("build_env variable is not in workspace {}".format(workspace_config["name"]))
    if "build_tool" not in workspace_config:
        raise Exception("build_tool variable is not in workspace {}".format(workspace_config["name"]))

    # Set up the tooling and environment
    base_dir = workspace_config["workspace_root"]
    build_env = jmbe.get_build_env(workspace_config["build_env"])(workspace_config)

    if list_options:
        options = build_env.getCommands()
        print("Workspace {} uses buildenv {} with commands:".format(workspace_config["name"], type(build_env)))
        for o in options:
            print("- {}".format(o))
        return

    # Get the build command to run
    enved_build_command = build_env.getCommand(command, ' '.join(args))

    # Execute!
    if enved_build_command is not None:
        print("Executing `{}` in `{}`".format(enved_build_command, base_dir))
        if not dry_run:
            subprocess.call(enved_build_command, cwd=base_dir, shell=True)

def workspace_build(workspace_config, args, dry_run=False):
    # check config for vars
    if "build_env" not in workspace_config:
        raise Exception("build_env variable is not in workspace {}".format(workspace_config["name"]))
    if "build_tool" not in workspace_config:
        raise Exception("build_tool variable is not in workspace {}".format(workspace_config["name"]))

    # Set up the tooling and environment
    base_dir = workspace_config["workspace_root"]
    build_env = jmbe.get_build_env(workspace_config["build_env"])(workspace_config)
    build_env_setup = build_env.setup()

    while True:
        if len(build_env_setup) <= 0:
            break
        m, a = build_env_setup.pop(0)
        if m == jmbe.SETUP.READY:
            break
        if m == jmbe.SETUP.COMMAND:
            print("Executing `{}` in `{}`".format(a, base_dir))
            if not dry_run:
                subprocess.call(a, cwd=base_dir, shell=True)
        if m == jmbe.SETUP.MESSAGE:
            print("Message from {}: {}".format(workspace_config["name"], a))
        if m == jmbe.SETUP.SETUP:
            build_env_setup = build_env.setup(a)
        if m == jmbe.SETUP.FAIL:
            print("Build Env Setup for {} failed: {}".format(workspace_config["name"], a))
            return

    build_tool = jmbt.get_build_tool(workspace_config["build_tool"])(workspace_config)

    # Get the build command to run
    build_command = build_tool.getBuildCommand(' '.join(args))
    enved_build_command = build_env.renderCommand(build_command)

    # Execute!
    print("Executing `{}` in `{}`".format(enved_build_command, base_dir))
    if not dry_run:
        subprocess.call(enved_build_command, cwd=base_dir, shell=True)