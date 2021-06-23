import copy
import os
import yaml

DEFAULT_CONFIG_FILE = ".jd"

def find_config(root_dir):
    """
    Finds a config from a root directory, searches parent directories
    for configs as well

    Parameters
    ----------
    root_dir : str
        directory to start the search at

    Returns
    -------
    str
        most local configuration
    """
    cur_dir = root_dir
    while True:
        cur_check = os.path.join(cur_dir, DEFAULT_CONFIG_FILE)
        if os.path.exists(cur_check):
            return cur_check
        cur_dir = os.path.split(cur_dir)[0]
        if cur_dir == "" or cur_dir =="/" or cur_dir == ".":
            # termination condition if there is nothing more to check
            raise Exception("No config found in the current directory or any parent directories.")

def process_workspaces_tag(raw_yaml, base_workspace, workspaces, source):
    """
    Process workspaces tag in raw_yaml

    Parameters
    ----------
    raw_yaml : dict
        yaml directly loaded from config file
    base_workspace : dict
        base_workspace from config if one exists
    workspaces : dict
        contains existing workspaces to check for duplication and also
        add new workspaces to
    source : str
        file that the raw_yaml is from
    """
    if "workspaces" in raw_yaml:
        for workspace in raw_yaml["workspaces"]:
            # verify no dupes
            if workspace in workspaces:
                raise Exception("Workspaces [{}] is duplicated from {} and {}}".format(w, workspaces[w]["workspace_source"], file))

            # extend new_workspace off of base
            base = copy.deepcopy(base_workspace)
            base["after"] = [] # we don't use dependencies from the base_workspace
            new_workspace = raw_yaml["workspaces"][workspace]

            for k in new_workspace:
                # if it's a list extend the values otherwise, set 
                if type(new_workspace[k]) is list and k in base:
                    base[k].extend(new_workspace[k])
                else:
                    base[k] = new_workspace[k]

            base["workspace_source"] = source

            if "root_directory" not in base:
                base["workspace_root"] = os.path.split(source)[0]
            else:
                base["workspace_root"] = os.path.join(os.path.split(source)[0], base["root_directory"])
            if base["workspace_root"] == "":
                base["workspace_root"] = "./"

            base["name"] = workspace
            workspaces[workspace] = base

def process_alises_tag(raw_yaml, alises, source):
    """
    Process aliases tag in raw_yaml

    Parameters
    ----------
    raw_yaml : dict
        yaml directly loaded from config file
    aliases : dict
        contains existing aliases to check for duplication and also
        add new workspaces to
    source : str
        file that the raw_yaml is from
    """
    if "aliases" in raw_yaml:
        for alias in raw_yaml["aliases"]:
            if alias not in config.aliases:
                config.aliases[alias] = {}
            else:
                print("Warning, alias {} from {} is being overwritten by {}".format(alias, config.aliases[alias]["source"], file_path))
            config.aliases[alias]["source"] = file_path
            config.aliases[alias]["to"] = raw_yaml["aliases"][alias]

class Config():
    """ 
    Config that is loaded from a yaml
    
    Attributes
    ----------
    workspaces : dict
        workspaces loaded from various yaml files
    aliases : dict
        aliases loaded from various yaml files
    """
    @staticmethod
    def from_call_dir(root_dir, call_dir = os.getcwd()):
        config_file = find_config(root_dir)
        return Config.from_config_file(config_file, call_dir)

    @staticmethod
    def from_config_file(config_file, call_dir = os.getcwd()):
        return process_workspace(config_file)

    def __init__(self):
        self.workspaces = {}
        self.aliases = {}

def process_workspace(file_path):
    base_ws_dir = os.path.split(file_path)[0]
    if base_ws_dir == "":
        base_ws_dir = "./"

    # Read in the yaml from a file
    raw_yaml = None
    with open(file_path, 'r') as fstream:
        raw_yaml = yaml.safe_load(fstream)
    if raw_yaml is None:
        raise Exception("error loading %s file")

    config = Config()
    # reduce dupe of same code for adding a subworkspace
    def process_sub_workspace(current_config, sub_ws_path):
        new_config = process_workspace(sub_ws_path)
        # verify no repeats! and add new workspaces
        for w in new_config.workspaces:
            if w in current_config.workspaces:
                raise Exception("Workspaces [{}] is duplicated from {} and {}".format(w, current_config.workspaces[w]["workspace_source"], new_config.workspaces[w]["workspace_source"]))
            current_config.workspaces[w] = new_config.workspaces[w]
        for a in new_config.aliases:
            if a == "all" or a == "default":
                continue
            if a in current_config.aliases:
                print("Warning: alias {1} is duplicated from {2} and {3}, selecting definition from {3}".format(a, current_config.aliases[a]["source"], new_config.aliases[a]["source"]))
            current_config.aliases[a] = new_config.aliases[a]

    # include any explicitly included sub workspaces
    if "include_workspaces" in raw_yaml:
        for sub_ws in raw_yaml["include_workspaces"]:
            sub_ws_path = os.path.join(base_ws_dir, sub_ws)
            process_sub_workspace(sub_ws_path)

    # Search all subdirs for workspaces and add them
    if "include_dir_search" in raw_yaml and raw_yaml["include_dir_search"]:
        subdirs = [ f.path for f in os.scandir(base_ws_dir) if f.is_dir() ]
        for subdir in subdirs:
            subdir_ws_path = os.path.join(subdir, DEFAULT_CONFIG_FILE)
            if os.path.exists(subdir_ws_path):
                process_sub_workspace(config, subdir_ws_path)

    # Get base workspace if it exists
    base_workspace = {}
    if "base_workspace" in raw_yaml:
        base_workspace = raw_yaml["base_workspace"]

    # add any workspaces specified in this config file
    process_workspaces_tag(raw_yaml, base_workspace, config.workspaces, file_path)
    process_alises_tag(raw_yaml, config.aliases, file_path)
    if "all" in config.workspaces:
        raise Exception("`all` must only be an alias and not a workspace (as defined in {}".format(file_path))
    if "default" in config.workspaces:
        raise Exception("`default` must only be an alias and not a workspace (as defined in {}".format(file_path))

    return config

