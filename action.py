from .actionRegistry import register_action, register_control_action
from .instructor import Validator, Executor
from concurrent.futures import ThreadPoolExecutor
from .timeSeries import TimeSeries

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
            reachyArm.gotoCartesianPoint(position, orientation, interpolation=interpolation)
    else:
        if interpolation == None:
            reachyArm.gotoCartesianPoint(position, orientation, duration=duration)
        else:
            reachyArm.gotoCartesianPoint(position, orientation, duration=duration, interpolation=interpolation)

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

    for _ in range(times):
        for action in actions:
            executor.executeInstruction(action)


@register_control_action("capture")
def capture(executor : "Executor", params : dict):
    if not Validator(params, "capture").require("as").require("action").validate():
        return
    

    varName : str = params.get("as")
    action = params.get("action")

    executor.variable[varName] = executor.executeInstruction(action)

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