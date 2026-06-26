from concurrent.futures import ThreadPoolExecutor
import time

from .actionRegistry import register_action, register_control_action
from .instructor import Validator, Executor, ExpressionEvaluator
from .timeSeries import TimeSeries


class BreakLoop(Exception):
    """Raised by the 'break' action to exit a loop."""
    pass


class ContinueLoop(Exception):
    """Raised by the 'continue' action to advance to the next loop iteration."""
    pass


@register_action("reachy_on")
def reachy_on(executor: "Executor"):
    """
    Turn on all Reachy motors.

    @param executor: Bound executor.
    @type executor: Executor
    """
    executor.reachy.turnOn()


@register_action("reachy_off")
def reachy_off(executor: "Executor"):
    """
    Turn off all Reachy motors smoothly.

    @param executor: Bound executor.
    @type executor: Executor
    """
    executor.reachy.turnOffSmooth()


@register_action("look_at")
def look_at(executor: "Executor", params: dict):
    """
    Point the head at a 3D target.

    Required: target (list[float]).
    Optional: duration (float).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "look_at").require("target").validate():
        return
    target   = params["target"]
    duration = params.get("duration")
    if duration is None:
        executor.reachy.head.lookAt(target)
    else:
        executor.reachy.head.lookAt(target, duration=duration)


@register_action("move_hand")
def move_hand(executor: "Executor", params: dict):
    """
    Move an arm end-effector to a Cartesian position.

    Required: arm ('left'|'right'), position (list[float]), orientation (list[float]).
    Optional: duration (float), interpolation (str).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "move_hand").require("arm").require("position").require("orientation").validate():
        return

    arm          = params["arm"]
    position     = params["position"]
    orientation  = params["orientation"]
    duration     = params.get("duration")
    interpolation = params.get("interpolation")

    reachyArm = executor.reachy.armLeft if arm == "left" else executor.reachy.armRight

    interp_obj = reachyArm.getInterpoaltionByName(interpolation) if interpolation is not None else None

    if duration is None and interp_obj is None:
        reachyArm.gotoCartesianPoint(position, orientation)
    elif duration is None:
        reachyArm.gotoCartesianPoint(position, orientation, interpolation=interp_obj)
    elif interp_obj is None:
        reachyArm.gotoCartesianPoint(position, orientation, duration=duration)
    else:
        reachyArm.gotoCartesianPoint(position, orientation, duration=duration, interpolation=interp_obj)


@register_action("place_hand_on_table")
def place_hand_on_table(executor: "Executor", params: dict):
    """
    Reset an arm to a neutral table-resting pose (debug, no collision checks).

    Required: arm ('left'|'right').
    Optional: duration (float).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "place_hand_on_table").require("arm").validate():
        return

    arm       = params["arm"]
    duration  = params.get("duration")
    reachyArm = executor.reachy.armLeft if arm == "left" else executor.reachy.armRight

    if duration is not None:
        reachyArm._debug_placeHandOnTable(duration)
    else:
        reachyArm._debug_placeHandOnTable()


@register_control_action("parallel")
def parallel(executor: "Executor", params: list):
    """
    Execute a list of actions in parallel using a thread pool.

    Required: params must be a list of action dicts.

    @param executor: Bound executor.
    @type executor: Executor
    @param params: List of action instructions.
    @type params: list
    """
    if not Validator(params, "parallel").isAList().validate():
        return
    with ThreadPoolExecutor(max_workers=len(params)) as pool:
        futures = [pool.submit(executor.executeInstruction, action) for action in params]
        for future in futures:
            future.result()


@register_control_action("do")
def do(executor: "Executor", params: dict):
    """
    Execute a list of actions a fixed number of times.

    Required: times (int), actions (list).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "do").require("times").require("actions").validate():
        return
    if not Validator(params["actions"], "do").isAList().validate():
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
    """
    Store a value or action result in a named variable.

    Required: as (str).
    Optional: value (any) or action (dict).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    @return: Captured value.
    """
    if not Validator(params, "capture").require("as").validate():
        return

    var_name = params["as"]

    if "value" in params:
        executor.variable[var_name] = ExpressionEvaluator.evaluate(executor, params["value"])
        return executor.variable[var_name]

    if "action" in params:
        result = executor.executeInstruction(params["action"])
        executor.variable[var_name] = result
        return result

    return None


@register_action("record_all")
def record_all(executor: "Executor", params: dict):
    """
    Record all parts simultaneously.

    Required: duration (float), fps (float).
    Optional: head (bool), arm_right (bool), arm_left (bool).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    @rtype: TimeSeries
    """
    if not Validator(params, "record_all").require("duration").require("fps").validate():
        return
    return executor.reachy.record(
        params["duration"], params["fps"],
        params.get("arm_left", True),
        params.get("arm_right", True),
        params.get("head", True),
    )


@register_action("record_head")
def record_head(executor: "Executor", params: dict):
    """
    Record head orientation only.

    Required: duration (float), fps (float).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    @rtype: TimeSeries
    """
    if not Validator(params, "record_head").require("duration").require("fps").validate():
        return
    return executor.reachy.head.recordHead(params["duration"], params["fps"])


@register_action("record_arm")
def record_arm(executor: "Executor", params: dict):
    """
    Record a single arm.

    Required: arm ('left'|'right'), duration (float), fps (float).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    @rtype: TimeSeries
    """
    if not Validator(params, "record_arm").require("arm").require("duration").require("fps").validate():
        return
    reachyArm = executor.reachy.armLeft if params["arm"] == "left" else executor.reachy.armRight
    return reachyArm.recordArm(params["duration"], params["fps"])


@register_action("save_record_as_CSV")
def save_record_as_CSV(executor: "Executor", params: dict):
    """
    Save a TimeSeries to CSV.

    Required: file_name (str), record (TimeSeries).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "save_record_as_CSV").require("file_name").require("record").validate():
        return
    params["record"].saveToCSV(params["file_name"])


@register_action("load_record_from_CSV")
def load_record_from_CSV(executor: "Executor", params: dict):
    """
    Load a TimeSeries from CSV.

    Required: file_name (str).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    @rtype: TimeSeries
    """
    if not Validator(params, "load_record_from_CSV").require("file_name").validate():
        return
    return TimeSeries.loadFromCSV(params["file_name"])


@register_action("save_record_as_JSON")
def save_record_as_JSON(executor: "Executor", params: dict):
    """
    Save a TimeSeries to JSON.

    Required: file_name (str), record (TimeSeries).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "save_record_as_JSON").require("file_name").require("record").validate():
        return
    params["record"].saveToJson(params["file_name"])


@register_action("load_record_from_JSON")
def load_record_from_JSON(executor: "Executor", params: dict):
    """
    Load a TimeSeries from JSON.

    Required: file_name (str).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    @rtype: TimeSeries
    """
    if not Validator(params, "load_record_from_JSON").require("file_name").validate():
        return
    return TimeSeries.loadFromJson(params["file_name"])


@register_action("open_hand")
def open_hand(executor: "Executor", params: dict):
    """
    Open the gripper of an arm.

    Required: arm ('left'|'right').
    Optional: duration (float).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "open_hand").require("arm").validate():
        return
    reachyArm = executor.reachy.armLeft if params["arm"] == "left" else executor.reachy.armRight
    duration  = params.get("duration")
    reachyArm.openHand() if duration is None else reachyArm.openHand(duration)


@register_action("close_hand")
def close_hand(executor: "Executor", params: dict):
    """
    Close the gripper of an arm.

    Required: arm ('left'|'right').
    Optional: duration (float).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "close_hand").require("arm").validate():
        return
    reachyArm = executor.reachy.armLeft if params["arm"] == "left" else executor.reachy.armRight
    duration  = params.get("duration")
    reachyArm.closeHand() if duration is None else reachyArm.closeHand(duration)


@register_action("play_record_all")
def play_record_all(executor: "Executor", params: dict):
    """
    Replay a TimeSeries on all parts.

    Required: record (TimeSeries).
    Optional: start_duration (float).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "play_record_all").require("record").validate():
        return
    record        = params["record"]
    startDuration = params.get("start_duration")
    if startDuration is None:
        executor.reachy.playRecord(record)
    else:
        executor.reachy.playRecord(record, startDuration)


@register_action("play_record_head")
def play_record_head(executor: "Executor", params: dict):
    """
    Replay a TimeSeries on the head only.

    Required: record (TimeSeries).
    Optional: start_duration (float).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "play_record_head").require("record").validate():
        return
    record        = params["record"]
    startDuration = params.get("start_duration")
    if startDuration is None:
        executor.reachy.head.playHeadRecord(record)
    else:
        executor.reachy.head.playHeadRecord(record, startDuration)


@register_action("play_record_arm")
def play_record_arm(executor: "Executor", params: dict):
    """
    Replay a TimeSeries on a single arm.

    Required: record (TimeSeries), arm ('left'|'right').
    Optional: start_duration (float), collision_check_number (int).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "play_record_arm").require("record").require("arm").validate():
        return

    record                 = params["record"]
    startDuration          = params.get("start_duration")
    collisionCheckInterval = params.get("collision_check_number")
    reachyArm              = executor.reachy.armLeft if params["arm"] == "left" else executor.reachy.armRight

    kwargs = {}
    if startDuration is not None:
        kwargs["startDuration"] = startDuration
    if collisionCheckInterval is not None:
        kwargs["collisionCheckInterval"] = collisionCheckInterval

    reachyArm.playArmRecord(record, **kwargs)


@register_action("move_joints")
def move_joints(executor: "Executor", params: dict):
    """
    Move an arm by specifying all joint angles directly.

    Required: arm ('left'|'right'), joints (list[float]), duration (float).
    Optional: interpolation (str), collision_check_number (int).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "move_joints").require("arm").require("joints").require("duration").validate():
        return

    arm                    = params["arm"]
    joints                 = params["joints"]
    duration               = params["duration"]
    collisionCheckInterval = params.get("collision_check_number")
    interpolation          = params.get("interpolation")
    reachyArm              = executor.reachy.armLeft if arm == "left" else executor.reachy.armRight

    jointsDict = {joint: value for joint, value in zip(reachyArm.getJointsInOrder(), joints)}

    kwargs = {}
    if collisionCheckInterval is not None:
        kwargs["steps"] = collisionCheckInterval
    if interpolation is not None:
        kwargs["interpolation"] = reachyArm.getInterpoaltionByName(interpolation)

    reachyArm.safeGoto(jointsDict, duration, **kwargs)


@register_action("get_joint_positions")
def get_joint_positions(executor: "Executor", params: dict):
    """
    Return the current joint positions for an arm.

    Required: arm ('left'|'right').

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    @return: Joint values view from the SDK.
    """
    if not Validator(params, "get_joint_position").require("arm").validate():
        return
    reachyArm = executor.reachy.armLeft if params["arm"] == "left" else executor.reachy.armRight
    return reachyArm._reachyArm.joints.values()


@register_action("print")
def print_action(executor: "Executor", params: dict):
    """
    Print a message at the specified level.

    Required: message (str).
    Optional: type ('safety'|'debug'|'warning'; default info).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "print").require("message").validate():
        return

    message   = "[PRINT ACTION] " + str(params["message"])
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
def get_look_position(executor: "Executor"):
    """
    Return the point the head is currently looking at.

    @param executor: Bound executor.
    @type executor: Executor
    @rtype: list[float]
    """
    return executor.reachy.head.forwardKinematic()


@register_action("get_hand_position")
def get_hand_position(executor: "Executor", params: dict):
    """
    Return the current end-effector position for an arm.

    Required: arm ('left'|'right').

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    @rtype: list[float]
    """
    if not Validator(params, "get_hand_position").require("arm").validate():
        return
    reachyArm = executor.reachy.armLeft if params["arm"] == "left" else executor.reachy.armRight
    return reachyArm.getHandPosition()


@register_action("condition")
def condition(executor: "Executor", params) -> bool:
    """
    Evaluate an expression and return its boolean value.

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Expression to evaluate.
    @rtype: bool
    """
    return bool(ExpressionEvaluator.evaluate(executor, params))


@register_control_action("if")
def if_action(executor: "Executor", params: dict):
    """
    Execute actions if the condition is true.

    Required: condition (expression), actions (list).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "if").require("condition").require("actions").validate():
        return
    if condition(executor, params["condition"]):
        for action in params["actions"]:
            executor.executeInstruction(action)


@register_control_action("if_else")
def if_else_action(executor: "Executor", params: dict):
    """
    Execute the 'then' branch if condition is true, otherwise the 'else' branch.

    Required: condition (expression), then (list), else (list).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "if_else").require("condition").require("then").require("else").validate():
        return
    branch = params["then"] if condition(executor, params["condition"]) else params["else"]
    for action in branch:
        executor.executeInstruction(action)


@register_control_action("while")
def while_action(executor: "Executor", params: dict):
    """
    Repeat actions while the condition is true.

    Required: condition (expression), actions (list).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    while condition(executor, params["condition"]):
        try:
            for action in params["actions"]:
                executor.executeInstruction(action)
        except ContinueLoop:
            continue
        except BreakLoop:
            break


@register_control_action("wait")
def wait_action(executor: "Executor", params: dict):
    """
    Pause execution for a given duration.

    Required: duration (float).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "wait").require("duration").validate():
        return
    time.sleep(params["duration"])


@register_control_action("break")
def break_action(executor: "Executor"):
    """
    Break out of the enclosing loop.

    @param executor: Bound executor.
    @type executor: Executor
    """
    raise BreakLoop()


@register_control_action("continue")
def continue_action(executor: "Executor"):
    """
    Skip to the next iteration of the enclosing loop.

    @param executor: Bound executor.
    @type executor: Executor
    """
    raise ContinueLoop()


@register_action("enable_table_collision")
def enable_table_collision(executor: "Executor", params: dict):
    """
    Enable table collision checking for an arm.

    Required: arm ('left'|'right').

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "enable_table_collision").require("arm").validate():
        return
    reachyArm = executor.reachy.armLeft if params["arm"] == "left" else executor.reachy.armRight
    reachyArm.activateCollisionWithTable()


@register_action("disable_table_collision")
def disable_table_collision(executor: "Executor", params: dict):
    """
    Disable table collision checking for an arm.

    Required: arm ('left'|'right').

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "disable_table_collision").require("arm").validate():
        return
    reachyArm = executor.reachy.armLeft if params["arm"] == "left" else executor.reachy.armRight
    reachyArm.desactivateCollisionWithTable()


@register_action("set_table")
def set_table(executor: "Executor", params: dict):
    """
    Define or update the table AABB collider.

    Required: x_min, x_max, y_min, y_max, z_min, z_max (float).
    Optional: arm ('right'|'left'|'both'; default 'both').

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not (Validator(params, "set_table")
            .require("x_min").require("x_max")
            .require("y_min").require("y_max")
            .require("z_min").require("z_max")
            .validate()):
        return

    from .tableCollider import TableCollider
    from . import consoleManager as cm

    try:
        table = TableCollider(
            params["x_min"], params["x_max"],
            params["y_min"], params["y_max"],
            params["z_min"], params["z_max"],
        )
    except ValueError as e:
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
    Define the table collider from its surface height and optional thickness.

    Required: x_min, x_max, y_min, y_max, z_surface (float).
    Optional: thickness (float; default 0.10), arm ('right'|'left'|'both'; default 'both').

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not (Validator(params, "set_table_from_surface")
            .require("x_min").require("x_max")
            .require("y_min").require("y_max")
            .require("z_surface")
            .validate()):
        return

    from .tableCollider import TableCollider
    from . import consoleManager as cm

    try:
        table = TableCollider.fromSurface(
            params["x_min"], params["x_max"],
            params["y_min"], params["y_max"],
            params["z_surface"],
            params.get("thickness", 0.10),
        )
    except ValueError as e:
        cm.MKprintWarning(str(e), "set_table_from_surface")
        return

    arm = params.get("arm", "both")
    if arm in ("right", "both"):
        executor.reachy.armRight.setTable(table)
    if arm in ("left", "both"):
        executor.reachy.armLeft.setTable(table)


@register_action("move_hand_sequence")
def move_hand_sequence(executor: "Executor", params: dict):
    """
    Cycle through a list of Cartesian positions for a given total duration.

    Required: arm ('left'|'right'), positions (list[list[float]]), duration (float).
    Optional: step_duration (float; default 0.5), orientation (list[float]; default [0,0,0]).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "move_hand_sequence").require("arm").require("positions").require("duration").validate():
        return

    arm            = params["arm"]
    positions      = params["positions"]
    total_duration = params["duration"]
    step_duration  = params.get("step_duration", 0.5)
    orientation    = params.get("orientation", [0, 0, 0])

    reachyArm = executor.reachy.armLeft if arm == "left" else executor.reachy.armRight

    start = time.time()
    i     = 0
    while True:
        elapsed   = time.time() - start
        remaining = total_duration - elapsed
        if remaining <= 0:
            break
        pos         = positions[i % len(positions)]
        actual_step = min(step_duration, remaining)
        reachyArm.gotoCartesianPoint(pos, orientation, duration=actual_step)
        i += 1


@register_action("set_antenna")
def set_antenna(executor: "Executor", params: dict):
    """
    Move an antenna to an absolute angle.

    Required: antenna ('left'|'right'), angle (float).
    Optional: duration (float; default 0.5).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "set_antenna").require("antenna").require("angle").validate():
        return

    antenna  = executor.reachy.head.antennaLeft if params["antenna"] == "left" else executor.reachy.head.antennaRight
    angle    = params["angle"]
    duration = params.get("duration", 0.5)
    antenna.setAntenna(angle, duration)


@register_action("vibrate_antenna")
def vibrate_antenna(executor: "Executor", params: dict):
    """
    Vibrate an antenna around its current position.

    Required: antenna ('left'|'right').
    Optional: amplitude (float; default 15.0), cycles (int; default 3), speed (float; default 0.08).

    @param executor: Bound executor.
    @type executor: Executor
    @param params: Action parameters.
    @type params: dict
    """
    if not Validator(params, "vibrate_antenna").require("antenna").validate():
        return

    antenna   = executor.reachy.head.antennaLeft if params["antenna"] == "left" else executor.reachy.head.antennaRight
    amplitude = params.get("amplitude", 15.0)
    cycles    = params.get("cycles", 3)
    speed     = params.get("speed", 0.08)
    antenna.vibrateAntenna(amplitude, cycles, speed)
