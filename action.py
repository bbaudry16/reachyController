from .actionRegistry import register

@register("reachy_on")
def reachy_on(executor):
    executor.reachy.turnOn()


@register("reachy_off")
def reachy_off(executor):
    executor.reachy.turnOffSmooth()

@register("look_at")
def look_at(executor, params):
    target = params["target"]
    executor.reachy.head.lookAt(target, 5)