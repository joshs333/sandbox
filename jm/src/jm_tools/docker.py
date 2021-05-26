import yaml

def compose_list_services(files=[]):
    result = []
    for file in files:
        try:
            with open(file, 'r') as fstream:
                y = yaml.safe_load(fstream)
                if "services" in y:
                    for k in y["services"]:
                        result.append(k)
        except Exception as err:
            print("Warning: unable to process file [%s] due to [%s]"%(file, str(err)))
    return result

def compose_base_command(service_name, files=[]):
    s = compose_list_services()
    if service_name not in s:
        raise Exception("service named: %s does not exist in provided files."%service_name)

    command = "docker-compose"
    for f in files:
        command += " -f " + f
    command += " -p service_name"
    return command

def compose_up(service_name, files=[]):
    return compose_base_command(service_name, files) + " up " + service_name

def compose_build(service_name, files=[]):
    return compose_base_command(service_name, files) + " build " + service_name