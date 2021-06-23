import os

# Why is docstring so verbose... bloated... and cluttery...
def process_workspace_list(arguments, config, calling_dir = os.getcwd()):
    """
    Processes a list of arguments from command line and resolves it to
    a list of workspaces.

    Parameters
    ----------
    arguments : list of str
        command line arguments to resolve to workspaces
    aliases : dict of alias info
        alias information from Config()
    calling_dir : str
        used to determine the default workspace if not explicitly specified

    Returns
    -------
    list of str
        a list of workspaces the arguments resolve to
    """
    def resolve_default():
        if len(config.workspaces.keys()) == 1:
            # Default to the single workspace that exists...
            return [w for w in config.workspaces.keys()]
        else:
            # Check if we can get a default based on what directory we are calling from :)
            abs_call_dir = os.path.abspath(calling_dir)
            def_results = []
            for w in config.workspaces:
                config_dir = os.path.abspath(config.workspaces[w]["workspace_root"])
                rel_path = os.path.relpath(abs_call_dir, config_dir)
                if ".." not in rel_path:
                    def_results.append(w)
            return def_results
    def resolve_all():
        return [k for k in config.workspaces.keys()]
    
    initial_workspaces = []
    processed_aliases = []
    final_workspaces = []

    resolvable_aliases = {"all": resolve_all, "default": resolve_default}

    def process_extension(workspaces, source=""):
        for ws in workspaces:
            if ws in initial_workspaces:
                continue
            if ws in processed_aliases:
                print("warn: arrived at alias [{}] again from [{}], possible loop or redundant args.".format(ws, source))
                continue
            if ws not in config.aliases and ws not in config.workspaces and ws not in resolvable_aliases:
                raise Exception("Error: unable to resolve [{}] to a worksapce or alias.".format(ws))
            initial_workspaces.append(ws)

    if len(arguments) <= 0:
        initial_workspaces.append("default")
    else:
        process_extension(arguments, "<arguments>")

    while len(initial_workspaces) > 0:
        ws_now = initial_workspaces.pop(0)
        if ws_now in config.aliases:
            process_extension(config.aliases[ws_now]["to"], ws_now)
            processed_aliases.append(ws_now)
        elif ws_now in resolvable_aliases:
            process_extension(resolvable_aliases[ws_now](), ws_now)
            processed_aliases.append(ws_now)
        else:
            # We will add the ws to the final workspaces
            if ws_now not in final_workspaces:
                final_workspaces.append(ws_now)
    return final_workspaces
