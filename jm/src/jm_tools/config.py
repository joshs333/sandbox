

def config_load(path):
    with open(file, 'r') as fstream:
        y = yaml.safe_load(fstream)
        return y

def config_list_workspaces(y):
    if "workspaces" not in y:
        raise Exception("No workpaces in provided config!")

    result = []
    for ws in "workspaces":
        result.append(ws)
    return result

def config_get_workspace_info(ws_name, y):
    if "workspaces" not in y:
        raise Exception("No workpaces in provided config!")
    if "root_dir" not in y:
        raise Exception("No root_dir in provided config!")
    if ws_name not in y["workspaces"]:
        raise Exception("Workspace [%s] not in config!"%ws_name)

    base_config = {}
    if "base_workspace" in y:
        base_config = y["base_workspace"]

    config = y["workspaces"][ws_name]
    def extend_base(name):
        if name in base_config and name in config:
            config[name].extend(base_config[name])
        elif name in base_config:
            config[name] = base_config[name]
        elif name not in config:
            raise Exception("No specified %s"%name)
    extend_base("compose_files")
    extend_base("env_files")

    if "catkin_root" not in config:
        raise Exception("Workspace %s does not have catkin_root specified"%ws_name)
    if "service_name" not in config:
        raise Exception("Workspace %s does not have service_name specified"%ws_name)

    config["root_dir"] = y["root_dir"]
    return config
