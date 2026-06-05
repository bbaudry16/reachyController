from .actionRegistry import register_action, register_control_action
from .instructor import Validator, Executor
from concurrent.futures import ThreadPoolExecutor
from .timeSeries import TimeSeries


@register_action(
    "reachy_on",
    params={},
    description="Turn Reachy on"
)
def reachy_on(executor: "Executor"):
    executor.reachy.turnOn()


@register_action(
    "reachy_off",
    params={},
    description="Turn Reachy off smoothly"
)
def reachy_off(executor: "Executor"):
    executor.reachy.turnOffSmooth()


@register_action(
    "look_at",
    params={
        "target": "vec3"
    },
    description="Make Reachy look at a 3D target"
)
def look_at(executor: "Executor", params):
    if not Validator(params, "look_at").require("target").validate():
        return

    executor.reachy.head.lookAt(params["target"], 5)


@register_action(
    "move_hand",
    params={
        "arm": {"type": "enum", "values": ["left", "right"]},
        "position": "vec3",
        "orientation": "vec3",
        "duration": "number?",
        "interpolation": "string?"
    }
)
def move_hand(executor: "Executor", params: dict):
    if not Validator(params, "move_hand").require("arm").require("position").require("orientation").validate():
        return

    arm = params["arm"]
    reachyArm = executor.reachy.armLeft if arm == "left" else executor.reachy.armRight

    position = params["position"]
    orientation = params["orientation"]
    duration = params.get("duration")
    interpolation = params.get("interpolation")

    if duration is None:
        if interpolation is None:
            reachyArm.gotoCartesianPoint(position, orientation)
        else:
            reachyArm.gotoCartesianPoint(position, orientation, interpolation=interpolation)
    else:
        if interpolation is None:
            reachyArm.gotoCartesianPoint(position, orientation, duration=duration)
        else:
            reachyArm.gotoCartesianPoint(position, orientation, duration=duration, interpolation=interpolation)


@register_action(
    "place_hand_on_table",
    params={
        "arm": {"type": "enum", "values": ["left", "right"]},
        "duration": "number?"
    }
)
def place_hand_on_table(executor: "Executor", params: dict):
    if not Validator(params, "place_hand_on_table").require("arm").validate():
        return

    arm = params["arm"]
    reachyArm = executor.reachy.armLeft if arm == "left" else executor.reachy.armRight

    duration = params.get("duration")

    if duration is None:
        reachyArm._debug_placeHandOnTable()
    else:
        reachyArm._debug_placeHandOnTable(duration)


@register_control_action(
    "parallel",
    params={"actions": "list"},
    description="Run actions in parallel"
)
def parallel(executor: "Executor", params: list):
    if not Validator(params, "parallel").isAList().validate():
        return

    with ThreadPoolExecutor(max_workers=len(params)) as pool:
        futures = [pool.submit(executor.executeInstruction, a) for a in params]
        for f in futures:
            f.result()


@register_control_action(
    "do",
    params={
        "times": "number",
        "actions": "list"
    }
)
def do(executor: "Executor", params: dict):
    if not Validator(params, "do").require("times").require("actions").validate():
        return

    for _ in range(params["times"]):
        for action in params["actions"]:
            executor.executeInstruction(action)


@register_control_action(
    "capture",
    params={
        "as": "string",
        "action": "instruction"
    }
)
def capture(executor: "Executor", params: dict):
    if not Validator(params, "capture").require("as").require("action").validate():
        return

    executor.variable[params["as"]] = executor.executeInstruction(params["action"])


@register_action(
    "record_all",
    params={
        "duration": "number",
        "fps": "number",
        "head": "bool?",
        "arm_right": "bool?",
        "arm_left": "bool?"
    }
)
def record_all(executor: "Executor", params: dict):
    if not Validator(params, "record_all").require("duration").require("fps").validate():
        return

    return executor.reachy.record(
        params["duration"],
        params["fps"],
        params.get("arm_left", True),
        params.get("arm_right", True),
        params.get("head", True)
    )


@register_action(
    "record_head",
    params={
        "duration": "number",
        "fps": "number"
    }
)
def record_head(executor: "Executor", params: dict):
    if not Validator(params, "record_head").require("duration").require("fps").validate():
        return

    return executor.reachy.head.recordHead(params["duration"], params["fps"])


@register_action(
    "record_arm",
    params={
        "arm": {"type": "enum", "values": ["left", "right"]},
        "duration": "number",
        "fps": "number"
    }
)
def record_arm(executor: "Executor", params: dict):
    if not Validator(params, "record_arm").require("arm").require("duration").require("fps").validate():
        return

    arm = params["arm"]
    reachyArm = executor.reachy.armLeft if arm == "left" else executor.reachy.armRight

    return reachyArm.recordArm(params["duration"], params["fps"])


@register_action(
    "save_record_as_CSV",
    params={
        "file_name": "string",
        "record": "record"
    }
)
def save_record_as_CSV(executor: "Executor", params: dict):
    params["record"].saveToCSV(params["file_name"])


@register_action(
    "load_record_from_CSV",
    params={
        "file_name": "string"
    }
)
def load_record_from_CSV(executor: "Executor", params: dict):
    return TimeSeries.loadFromCSV(params["file_name"])


@register_action(
    "save_record_as_JSON",
    params={
        "file_name": "string",
        "record": "record"
    }
)
def save_record_as_JSON(executor: "Executor", params: dict):
    params["record"].saveToJson(params["file_name"])


@register_action(
    "load_record_from_JSON",
    params={
        "file_name": "string"
    }
)
def load_record_from_JSON(executor: "Executor", params: dict):
    return TimeSeries.loadFromJson(params["file_name"])


@register_action(
    "open_hand",
    params={
        "arm": {"type": "enum", "values": ["left", "right"]},
        "duration": "number?"
    }
)
def open_hand(executor: "Executor", params: dict):
    arm = params["arm"]
    reachyArm = executor.reachy.armLeft if arm == "left" else executor.reachy.armRight

    if params.get("duration") is None:
        reachyArm.openHand()
    else:
        reachyArm.openHand(params["duration"])


@register_action(
    "close_hand",
    params={
        "arm": {"type": "enum", "values": ["left", "right"]},
        "duration": "number?"
    }
)
def close_hand(executor: "Executor", params: dict):
    arm = params["arm"]
    reachyArm = executor.reachy.armLeft if arm == "left" else executor.reachy.armRight

    if params.get("duration") is None:
        reachyArm.closeHand()
    else:
        reachyArm.closeHand(params["duration"])


@register_action(
    "play_record_all",
    params={
        "record": "record",
        "start_duration": "number?"
    }
)
def play_record_all(executor: "Executor", params: dict):
    if params.get("start_duration") is None:
        executor.reachy.playRecord(params["record"])
    else:
        executor.reachy.playRecord(params["record"], params["start_duration"])


@register_action(
    "play_record_head",
    params={
        "record": "record",
        "start_duration": "number?"
    }
)
def play_record_head(executor: "Executor", params: dict):
    if params.get("start_duration") is None:
        executor.reachy.head.playHeadRecord(params["record"])
    else:
        executor.reachy.head.playHeadRecord(params["record"], params["start_duration"])


@register_action(
    "play_record_arm",
    params={
        "record": "record",
        "arm": {"type": "enum", "values": ["left", "right"]},
        "start_duration": "number?",
        "collision_check_number": "number?"
    }
)
def play_record_arm(executor: "Executor", params: dict):
    arm = params["arm"]
    reachyArm = executor.reachy.armLeft if arm == "left" else executor.reachy.armRight

    if params.get("start_duration") is None:
        if params.get("collision_check_number") is None:
            reachyArm.playArmRecord(params["record"])
        else:
            reachyArm.playArmRecord(params["record"], collisionCheckInterval=params["collision_check_number"])
    else:
        if params.get("collision_check_number") is None:
            reachyArm.playArmRecord(params["record"], startDuration=params["start_duration"])
        else:
            reachyArm.playArmRecord(
                params["record"],
                startDuration=params["start_duration"],
                collisionCheckInterval=params["collision_check_number"]
            )