from .actionRegistry import register_action, register_control_action
from .instructor import Validator, Executor, ExpressionEvaluator
from concurrent.futures import ThreadPoolExecutor
from .timeSeries import TimeSeries
import time

class BreakLoop(Exception):
    pass

class ContinueLoop(Exception):
    pass

@register_action("reachy_on")
def reachy_on(executor : "Executor"):
    executor.reachy.turnOn()


@register_action("reachy_off")
def reachy_off(executor : "Executor"):
    executor.reachy.turnOffSmooth()

@register_action("look_at")
def look_at(executor : "Executor", params):
    if not Validator(params, "look_at").require("target").validate():
        return
        
    target = params["target"]
    executor.reachy.head.lookAt(target, 5)

@register_action("move_hand")
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
            interpolationObject = reachyArm.getInterpoaltionByName(interpolation)
            reachyArm.gotoCartesianPoint(position, orientation, interpolation=interpolationObject)
    else:
        if interpolation == None:
            reachyArm.gotoCartesianPoint(position, orientation, duration=duration)
        else:
            interpolationObject = reachyArm.getInterpoaltionByName(interpolation)
            reachyArm.gotoCartesianPoint(position, orientation, duration=duration, interpolation=interpolationObject)

@register_action("place_hand_on_table")
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

@register_control_action("parallel")
def parallel(executor: "Executor", params: list):
    if not Validator(params, "parallel").isAList().validate():
        return

    with ThreadPoolExecutor(max_workers=len(params)) as pool:
        futures = [pool.submit(executor.executeInstruction, action) for action in params]

        for future in futures:
            future.result()


@register_control_action("do")
def do(executor: "Executor", params: dict):
    if not Validator(params, "do").require("times").require("actions").validate():
        return

    times = params["times"]
    actions = params["actions"]

    if not Validator(actions, "do").isAList().validate():
        return

    for _ in range(params["times"]):

        try:

            for action in params["actions"]:
                executor.executeInstruction(action)

        except ContinueLoop:
            continue

        except BreakLoop:
            break


@register_control_action("capture")
def capture(executor: "Executor", params: dict):

    if not Validator(params, "capture").require("as").validate():
        return

    var_name = params["as"]
    if "value" in params:
        executor.variable[var_name] = ExpressionEvaluator.evaluate(
            executor,
            params["value"]
        )
        return executor.variable[var_name]

    if "action" in params:
        result = executor.executeInstruction(params["action"])
        executor.variable[var_name] = result
        return result

    return None

@register_action("record_all")
def record_all(executor : "Executor", params : dict):
    if not Validator(params, "record_all").require("duration").require("fps").validate():
        return
    
    duration = params.get("duration")
    frequency = params.get("fps")

    head = params.get("head", True)
    arm_right = params.get("arm_right", True)
    arm_left = params.get("arm_left", True)

    return executor.reachy.record(duration, frequency, arm_left, arm_right, head)

@register_action("record_head")
def record_head(executor : "Executor", params : dict):
    if not Validator(params, "record_head").require("duration").require("fps").validate():
        return
    
    duration = params.get("duration")
    frequency = params.get("fps")

    return executor.reachy.head.recordHead(duration, frequency)

@register_action("record_arm")
def record_arm(executor : "Executor", params : dict):
    if not Validator(params, "record_arm").require("arm").require("duration").require("fps").validate():
        return
    
    duration = params.get("duration")
    frequency = params.get("fps")
    arm = params.get("arm")

    reachyArm = executor.reachy.armRight

    if(arm == "left"):
        reachyArm = executor.reachy.armLeft

    return reachyArm.recordArm(duration, frequency)

@register_action("save_record_as_CSV")
def save_record_as_CSV(executor : "Executor", params : dict):
    if not Validator(params, "save_record_as_CSV").require("file_name").require("record").validate():
        return
    
    fileName = params.get("file_name")
    record = params.get("record")

    record.saveToCSV(fileName)

@register_action("load_record_from_CSV")
def load_record_from_CSV(executor : "Executor", params : dict):
    if not Validator(params, "load_record_from_CSV").require("file_name").validate():
        return
    
    fileName = params.get("file_name")

    return TimeSeries.loadFromCSV(fileName)

@register_action("save_record_as_JSON")
def save_record_as_JSON(executor : "Executor", params : dict):
    if not Validator(params, "save_record_as_JSON").require("file_name").require("record").validate():
        return
    
    fileName = params.get("file_name")
    record : "TimeSeries"= params.get("record")

    record.saveToJson(fileName)

@register_action("load_record_from_JSON")
def load_record_from_JSON(executor : "Executor", params : dict):
    if not Validator(params, "load_record_from_JSON").require("file_name").validate():
        return
    
    fileName = params.get("file_name")

    return TimeSeries.loadFromJson(fileName)

@register_action("open_hand")
def open_hand(executor : "Executor", params : dict):
    if not Validator(params, "open_hand").require("arm").validate():
        return

    duration = params.get("duration")
    arm = params.get("arm")
    reachyArm = executor.reachy.armRight

    if(arm == "left"):
        reachyArm = executor.reachy.armLeft

    if duration is None:
        reachyArm.openHand()
    else:
        reachyArm.openHand(duration)


@register_action("close_hand")
def close_hand(executor : "Executor", params : dict):
    if not Validator(params, "close_hand").require("arm").validate():
        return

    duration = params.get("duration")
    arm = params.get("arm")
    reachyArm = executor.reachy.armRight

    if(arm == "left"):
        reachyArm = executor.reachy.armLeft

    if duration is None:
        reachyArm.closeHand()
    else:
        reachyArm.closeHand(duration)

@register_action("play_record_all")
def play_record_all(executor : "Executor", params : dict):
    if not Validator(params, "play_record_all").require("record").validate():
        return

    record : "TimeSeries" = params.get("record")
    startDuration = params.get("start_duration")

    if startDuration is None:
        executor.reachy.playRecord(record)
    else:
        executor.reachy.playRecord(record, startDuration)


@register_action("play_record_head")
def play_record_head(executor : "Executor", params : dict):
    if not Validator(params, "play_record_head").require("record").validate():
        return

    record : "TimeSeries" = params.get("record")
    startDuration = params.get("start_duration")

    if startDuration is None:
        executor.reachy.head.playHeadRecord(record)
    else:
        executor.reachy.head.playHeadRecord(record, startDuration)

@register_action("play_record_arm")
def play_record_arm(executor : "Executor", params : dict):
    if not Validator(params, "play_record_arm").require("record").require("arm").validate():
        return

    record : "TimeSeries" = params.get("record")
    startDuration = params.get("start_duration")
    collisionCheckInterval = params.get("collision_check_number")

    arm = params.get("arm")
    reachyArm = executor.reachy.armRight

    if(arm == "left"):
        reachyArm = executor.reachy.armLeft

    if startDuration is None:
        if collisionCheckInterval is None:
            reachyArm.playArmRecord(record)
        else:
            reachyArm.playArmRecord(record, collisionCheckInterval=collisionCheckInterval)
    else:
        if collisionCheckInterval is None:
            reachyArm.playArmRecord(record, startDuration=startDuration)
        else:
            reachyArm.playArmRecord(record, startDuration=startDuration, collisionCheckInterval=collisionCheckInterval)

@register_action("move_joints")
def move_joints(executor : "Executor", params : dict):
    if not Validator(params, "move_joints").require("arm").require("joints").require("duration").validate():
        return
    
    arm = params.get("arm")
    joints = params.get("joints")
    duration = params.get("duration")

    collisionCheckInterval = params.get("collision_check_number")
    interpolation = params.get("interpolation")

    reachyArm = executor.reachy.armRight

    if arm == "left":
        reachyArm = executor.reachy.armLeft

    jointsDict : dict = {joint : value for joint, value in zip(reachyArm.getJointsInOrder(), joints)}

    if interpolation is None:
        if collisionCheckInterval is None:
            reachyArm.safeGoto(jointsDict, duration)
        else:
            reachyArm.safeGoto(jointsDict, duration, steps=collisionCheckInterval)
    else:
        interpolationObject = reachyArm.getInterpoaltionByName(interpolation)
        if collisionCheckInterval is None:
            reachyArm.safeGoto(jointsDict, duration, interpolation=interpolationObject)
        else:
            reachyArm.safeGoto(jointsDict, duration, interpolation=interpolationObject, steps=collisionCheckInterval)

@register_action("get_joint_positions")
def get_joint_positions(executor : "Executor", params : dict):
    if not Validator(params, "get_joint_position").require("arm").validate():
        return

    arm = params.get("arm")

    reachyArm = executor.reachy.armRight

    if arm == "left":
        reachyArm = executor.reachy.armLeft

    return reachyArm._reachyArm.joints.values()

@register_action("print")
def print(executor : "Executor", params : dict):
    if not Validator(params, "print").require("message").validate():
        return

    message = "[PRINT ACTION] " + str(params.get("message"))
    printType = params.get("type")

    if printType == "safety":
        executor.printSafety(message)
    elif printType == "debug":
        executor.printDebug(message)
    elif printType == "warning":
        executor.printWarning(message)
    else:
        executor.print(message)


@register_action("get_look_position")
def get_look_position(executor : "Executor"):
    return executor.reachy.head.forwardKinematic()

@register_action("get_hand_position")
def get_hand_position(executor : "Executor", params : dict):
    if not Validator(params, "get_hand_position").require("arm").validate():
        return
    
    arm = params.get("arm")

    reachyArm = executor.reachy.armRight

    if arm == "left":
        reachyArm = executor.reachy.armLeft
    
    return reachyArm.getHandPosition()

@register_action("condition")
def condition(executor: "Executor", params: dict) -> bool:
    return bool(ExpressionEvaluator.evaluate(executor,params))

@register_control_action("if")
def if_action(executor, params):

    if not Validator(params, "if").require("condition").require("actions").validate():
        return

    if condition(executor, params["condition"]):
        for action in params["actions"]:
            executor.executeInstruction(action)

@register_control_action("if_else")
def if_else_action(executor, params):

    if not Validator(params, "if_else").require("condition").require("then").require("else").validate():
        return
    
    if condition(executor, params["condition"]):
        for action in params["then"]:
            executor.executeInstruction(action)
    else:
        for action in params["else"]:
            executor.executeInstruction(action)

@register_control_action("while")
def while_action(executor, params):

    while condition(executor, params["condition"]):
        try:
            for action in params["actions"]:
                executor.executeInstruction(action)

        except ContinueLoop:
            continue

        except BreakLoop:
            break

@register_control_action("wait")
def wait_action(executor, params):

    if not Validator(params, "wait").require("duration").validate():
        return

    time.sleep(params["duration"])

@register_control_action("break")
def break_action(executor):
    raise BreakLoop()

@register_control_action("continue")
def continue_action(executor):
    raise ContinueLoop()

@register_action("enable_table_collision")
def enable_table_collision(executor:"Executor", params:dict):
    if not Validator(params, "enable_table_collision").require("arm").validate():
        return
    
    arm = params.get("arm")

    reachyArm = executor.reachy.armRight

    if arm == "left":
        reachyArm = executor.reachy.armLeft
    
    reachyArm.activateCollisionWithTable()

@register_action("disable_table_collision")
def disable_table_collision(executor:"Executor", params:dict):
    if not Validator(params, "disable_table_collision").require("arm").validate():
        return
    
    arm = params.get("arm")

    reachyArm = executor.reachy.armRight

    if arm == "left":
        reachyArm = executor.reachy.armLeft
    
    reachyArm.desactivateCollisionWithTable()