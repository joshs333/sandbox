
class Arguments():
    positional_args = []
    flags = []
    set_args = {}

def parse_args(arguments, arg_flags=[]):
    pre_args = Arguments()
    post_args = []

    idx = 0
    # Process each argument
    while idx < len(arguments):
        arg = arguments[idx]
        next_arg = None
        ## Terminate at an alone -- and treat the rest as post_args
        if arg == "--" and len(arg) == 2:
            post_args = arguments[idx + 1:]
            break
        if idx + 1 < len(arguments):
            next_arg = arguments[idx + 1]
            if next_arg == "--" and len(next_arg) == 2:
                next_arg = None

        # Process an argument that has a - (a set or flag arg)
        def process_set(arg_t):
            # If there is an equal then it is a set argument
            if "=" in arg_t:
                split_arg = arg_t.split("=")
                pre_args.set_args[split_arg[0]] = split_arg[1]
                return 0
            # If it is in arg_flags then we can expect the next arg to be the set argument
            elif arg_t in arg_flags:
                if next_arg is None:
                    raise Exception("Excpected argument to flag {}".format(arg_t))
                pre_args.set_args[arg_t] = next_arg
                return 1
            # Else it's just a flag
            pre_args.flags.append(arg_t)
            return 0

        # Handle set / flag args
        if arg[:2] == "--":
            idx += process_set(arg[2:])
        elif arg[0] == "-":
            idx += process_set(arg[1:])
        # Else it's just a position arg!
        else:
            pre_args.positional_args.append(arg)
        idx = idx + 1
    return pre_args, post_args