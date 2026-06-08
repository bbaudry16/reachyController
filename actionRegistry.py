#registry where all action are stored actionName : callable
ACTION_REGISTRY = {}
#same for control action like do, while ect..
CONTROL_ACTIONS = []

#decorator to register action use : @register_action("name")
def register_action(name):
    def decorator(func):
        ACTION_REGISTRY[name] = func
        return func
    return decorator

#decorator to register control action use : @register_control_action("name")
def register_control_action(name):
    def decorator(func):
        ACTION_REGISTRY[name] = func
        CONTROL_ACTIONS.append(name)
        return func
    return decorator