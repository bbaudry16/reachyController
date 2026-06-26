ACTION_REGISTRY : dict = {}
CONTROL_ACTIONS : list = []


def register_action(name: str):
    """
    Decorator that registers a function as an action under the given name.

    @param name: Action name used as the key in L{ACTION_REGISTRY}.
    @type name: str
    @return: Decorator function.
    """
    def decorator(func):
        ACTION_REGISTRY[name] = func
        return func
    return decorator


def register_control_action(name: str):
    """
    Decorator that registers a function as a control-flow action.

    The function is added to both L{ACTION_REGISTRY} and L{CONTROL_ACTIONS}.

    @param name: Action name.
    @type name: str
    @return: Decorator function.
    """
    def decorator(func):
        ACTION_REGISTRY[name] = func
        CONTROL_ACTIONS.append(name)
        return func
    return decorator
