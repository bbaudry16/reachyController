from .actionRegistry import register
from .instructor import Validator, Executor
from concurrent.futures import ThreadPoolExecutor

@register("reachy_on")
def reachy_on(executor : "Executor"):
    executor.reachy.turnOn()


@register("reachy_off")
def reachy_off(executor : "Executor"):
    executor.reachy.turnOffSmooth()

@register("look_at")
def look_at(executor : "Executor", params):
    if not Validator(params, "look_at").require("target").validate():
        return
        
    target = params["target"]
    executor.reachy.head.lookAt(target, 5)

@register("move_hand")
def move_hand(executor : "Executor", params : dict):
    if not Validator(params, "move_hand").require("arm").require("position").require("orientation").validate():
        return
    
    arm = params["arm"]
    position = params["position"]
    orientation = params["orientation"]
    duration = params.get("duration")
    interpolation = params.get("interpolation")

    reachyArm = executor.reachy.armRight

    if(arm == "left"):
        reachyArm = executor.reachy.armLeft

    if duration == None:
        if interpolation == None:
            reachyArm.gotoCartesianPoint(position, orientation)
        else:
            reachyArm.gotoCartesianPoint(position, orientation, interpolation=interpolation)
    else:
        if interpolation == None:
            reachyArm.gotoCartesianPoint(position, orientation, duration=duration)
        else:
            reachyArm.gotoCartesianPoint(position, orientation, duration=duration, interpolation=interpolation)

@register("place_hand_on_table")
def place_hand_on_table(executor : "Executor", params : dict):
    if not Validator(params, "place_hand_on_table").require("arm").validate():
        return
    
    arm = params["arm"]
    duration = params.get("duration")

    reachyArm = executor.reachy.armRight

    if(arm == "left"):
        reachyArm = executor.reachy.armLeft
    
    if duration != None:
        reachyArm._debug_placeHandOnTable(duration)
    else:
        reachyArm._debug_placeHandOnTable()

@register("parallel")
def parallel(executor: "Executor", params: list):
    if not Validator(params, "parallel").isAList().validate():
        return

    with ThreadPoolExecutor(max_workers=len(params)) as pool:
        futures = [pool.submit(executor.executeInstruction, action) for action in params]

        for future in futures:
            future.result()


@register("do")
def do(executor: "Executor", params: dict):
    if not Validator(params, "do").require("times").require("actions").validate():
        return

    times = params["times"]
    actions = params["actions"]

    if not Validator(actions, "do").isAList().validate():
        return

    for _ in range(times):
        for action in actions:
            executor.executeInstruction(action)


