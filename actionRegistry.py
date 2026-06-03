ACTION_REGISTRY = {}

def register(name):
    def decorator(func):
        ACTION_REGISTRY[name] = func
        return func
    return decorator

