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
    duration = params.get("duration")
    if duration is None:
        executor.reachy.head.lookAt(target)
    else :
        executor.reachy.head.lookAt(target, duration=duration)

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
@register_action("set_table")
def set_table(executor: "Executor", params: dict):
    """
    Définit (ou met à jour) la table de travail comme une boîte AABB.
    Paramètres obligatoires :
        x_min, x_max : profondeur (avant du robot = X positif)
        y_min, y_max : largeur
        z_min, z_max : hauteur (z_max = surface de la table)
    Paramètre optionnel :
        arm : "right" | "left" | "both" (défaut : "both")
    """
    v = Validator(params, "set_table")
    if not v.require("x_min").require("x_max").require("y_min").require("y_max").require("z_min").require("z_max").validate():
        return

    from .tableCollider import TableCollider
    try:
        table = TableCollider(
            params["x_min"], params["x_max"],
            params["y_min"], params["y_max"],
            params["z_min"], params["z_max"],
        )
    except ValueError as e:
        from . import consoleManager as cm
        cm.MKprintWarning(str(e), "set_table")
        return

    arm = params.get("arm", "both")
    if arm in ("right", "both"):
        executor.reachy.armRight.setTable(table)
    if arm in ("left", "both"):
        executor.reachy.armLeft.setTable(table)


@register_action("set_table_from_surface")
def set_table_from_surface(executor: "Executor", params: dict):
    """
    Raccourci : définit la table par sa surface (z_surface) et son épaisseur.
    Paramètres obligatoires :
        x_min, x_max, y_min, y_max : empreinte au sol
        z_surface                  : hauteur de la surface (dessus de la table)
    Paramètre optionnel :
        thickness : épaisseur du plateau en m (défaut : 0.10)
        arm       : "right" | "left" | "both" (défaut : "both")
    """
    v = Validator(params, "set_table_from_surface")
    if not v.require("x_min").require("x_max").require("y_min").require("y_max").require("z_surface").validate():
        return

    from .tableCollider import TableCollider
    thickness = params.get("thickness", 0.10)
    try:
        table = TableCollider.fromSurface(
            params["x_min"], params["x_max"],
            params["y_min"], params["y_max"],
            params["z_surface"],
            thickness,
        )
    except ValueError as e:
        from . import consoleManager as cm
        cm.MKprintWarning(str(e), "set_table_from_surface")
        return

    arm = params.get("arm", "both")
    if arm in ("right", "both"):
        executor.reachy.armRight.setTable(table)
    if arm in ("left", "both"):
        executor.reachy.armLeft.setTable(table)

@register_action("move_hand_sequence")
def move_hand_sequence(executor, params):
    if not Validator(params, "move_hand_sequence").require("arm").require("positions").require("duration").validate():
        return
    
    arm = params["arm"]
    positions = params["positions"]
    total_duration = params["duration"]
    step_duration = params.get("step_duration", 0.5)
    orientation = params.get("orientation", [0, 0, 0])

    reachyArm = executor.reachy.armRight
    if arm == "left":
        reachyArm = executor.reachy.armLeft

    start = time.time()
    i = 0
    while (time.time() - start) < total_duration:
        pos = positions[i % len(positions)]
        reachyArm.gotoCartesianPoint(pos, orientation, duration=step_duration)
        time.sleep(step_duration)
        i += 1

@register_action("set_antenna")
def set_antenna(executor: "Executor", params: dict):
    if not Validator(params, "set_antenna").require("antenna").require("angle").validate():
        return
    antenna  = params.get("antenna")   # "left" or "right"
    angle    = params.get("angle")     # degrees
    duration = params.get("duration", 0.5)
    executor.reachy.head.setAntenna(antenna, angle, duration)

@register_action("vibrate_antenna")
def vibrate_antenna(executor: "Executor", params: dict):
    if not Validator(params, "vibrate_antenna").require("antenna").validate():
        return
    antenna   = params.get("antenna")
    amplitude = params.get("amplitude", 15.0)
    cycles    = params.get("cycles", 3)
    speed     = params.get("speed", 0.08)
    executor.reachy.head.vibrateAntenna(antenna, amplitude, cycles, speed)