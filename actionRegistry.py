ACTION_REGISTRY = {}
CONTROL_ACTIONS = set()
ACTION_META = {}
CONTROL_META = {}

def register_action(name, params=None, returns=None, description=None):
    def decorator(func):
        ACTION_REGISTRY[name] = func

        ACTION_META[name] = {
            "type": "action",
            "params": params or {},
            "returns": returns,
            "description": description or func.__doc__
        }

        return func
    return decorator


def register_control_action(name, params=None, description=None):
    def decorator(func):
        ACTION_REGISTRY[name] = func
        CONTROL_ACTIONS.add(name)

        CONTROL_META[name] = {
            "type": "control",
            "params": params or {},
            "description": description or func.__doc__
        }

        return func
    return decorator