ACTION_REGISTRY = {}
CONTROL_ACTIONS = []

def register_action(name):
    def decorator(func):
        ACTION_REGISTRY[name] = func
        return func
    return decorator

def register_control_action(name):
    def decorator(func):
        ACTION_REGISTRY[name] = func
        CONTROL_ACTIONS.append(name)
        return func
    return decorator