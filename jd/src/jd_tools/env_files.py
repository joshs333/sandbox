from dotenv import dotenv_values
import expandvars
import os

def get_vars(files = [], base_env=os.environ):
    """
    Gets environment variables from various dotenv files

    Expands environment variables in the filepath and in
    variables from the dotenv files
    """
    env = {}
    env_src = {}
    comb_env = base_env
    for file in files:
        file_exp = expandvars.expand(file, environ=comb_env)
        new_env = dotenv_values(file_exp)

        for k in new_env:
            v_exp = expandvars.expand(new_env[k], environ=comb_env)
            comb_env[k] = v_exp
            env[k] = v_exp
            env_src[k] = file_exp

    return env, env_src

