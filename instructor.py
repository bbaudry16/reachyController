import yaml as yml
import math

from . import consoleManager as cm
from . import reachyController
from .actionRegistry import ACTION_REGISTRY, CONTROL_ACTIONS


SCRIPT_NAME  : str = "Instructor"
SCRIPT_COLOR : str = cm.Color.BLUE


class Instructor:
    """
    Loads and executes YAML instruction sequences on a ReachyController.

    Accepts two file formats:
      - Standard YAML list of action dicts.
      - Reachy Instruction format (ryi): a YAML dict with a top-level
        'format: reachy_instruction' key and a 'reachy' list.

    @ivar data: List of instruction dicts to execute.
    @ivar executor: Executor instance bound to the controller.
    """

    def __init__(self, data: dict, reachyController: "reachyController.ReachyController"):
        """
        @param data: List of instruction dicts.
        @type data: list
        @param reachyController: Robot controller to execute instructions on.
        @type reachyController: reachyController.ReachyController
        """
        self.data     = data
        self.executor = Executor(reachyController)

    def execute(self) -> None:
        """Execute all instructions in order."""
        cm.MKprint("Executing instructions.", SCRIPT_NAME, SCRIPT_COLOR)
        cm.addIndentation()
        for instruction in self.data:
            self.executor.executeInstruction(instruction)
        cm.removeIndentation()
        cm.MKprint("Execution complete.", SCRIPT_NAME, SCRIPT_COLOR)

    @classmethod
    def loadFromPath(cls, path: str,
                     reachyController: "reachyController.ReachyController") -> "Instructor":
        """
        Load an Instructor from a YAML or ryi file path.

        @param path: Path to the instruction file.
        @type path: str
        @param reachyController: Robot controller to bind.
        @type reachyController: reachyController.ReachyController
        @rtype: Instructor
        """
        try:
            with open(path, 'r') as stream:
                load = yml.safe_load(stream)

            if (isinstance(load, dict)
                    and Validator(load).require("format").require("reachy").validate()
                    and load.get("format") == "reachy_instruction"
                    and isinstance(load.get("reachy"), list)):
                cm.MKprint(f"Loaded ryi format: {path}", SCRIPT_NAME, SCRIPT_COLOR)
                return cls(load.get("reachy"), reachyController)

            if isinstance(load, list):
                cm.MKprint(f"Loaded YAML format: {path}", SCRIPT_NAME, SCRIPT_COLOR)
                return cls(load, reachyController)

            raise ValueError("Invalid file format.")

        except Exception as e:
            cm.MKprintWarning(f"Failed to load {path}: {e}", SCRIPT_NAME, SCRIPT_COLOR)
            return cls({}, reachyController)

    @classmethod
    def loadFromString(cls, yamlString: str,
                       reachyController: "reachyController.ReachyController") -> "Instructor":
        """
        Load an Instructor from a YAML string.

        If the string does not start with 'reachy:', it is wrapped automatically.

        @param yamlString: YAML content as a string.
        @type yamlString: str
        @param reachyController: Robot controller to bind.
        @type reachyController: reachyController.ReachyController
        @rtype: Instructor
        """
        try:
            stripped = yamlString.strip()
            if not stripped.startswith("reachy:"):
                yamlString = "reachy:\n" + "\n".join(
                    "- " + line if line and not line.startswith("-") and not line.startswith(" ")
                    else line
                    for line in stripped.splitlines()
                )

            load = yml.safe_load(yamlString)

            if isinstance(load, dict) and "reachy" in load and isinstance(load.get("reachy"), list):
                cm.MKprint("Loaded ryi format from string.", SCRIPT_NAME, SCRIPT_COLOR)
                return cls(load.get("reachy"), reachyController)

            if isinstance(load, list):
                cm.MKprint("Loaded YAML format from string.", SCRIPT_NAME, SCRIPT_COLOR)
                return cls(load, reachyController)

            cm.MKprintWarning("Unknown format in string.", SCRIPT_NAME, SCRIPT_COLOR)
            return cls([], reachyController)

        except Exception as e:
            cm.MKprintWarning(f"Failed to load from string: {e}", SCRIPT_NAME, SCRIPT_COLOR)
            return cls([], reachyController)


class Executor:
    """
    Resolves and dispatches YAML action instructions to registered handlers.

    @ivar reachy: Bound robot controller.
    @ivar registry: Reference to the global action registry.
    @ivar variable: Runtime variable store for capture/resolve operations.
    """

    def __init__(self, reachy: "reachyController.ReachyController"):
        """
        @param reachy: Robot controller to execute actions on.
        @type reachy: reachyController.ReachyController
        """
        self.reachy    = reachy
        self.registry  = ACTION_REGISTRY
        self.variable  = {}

    def resolveExpressions(self, obj):
        """
        Recursively evaluate expressions in an object.

        @param obj: Value or nested structure containing expressions.
        @return: Evaluated result.
        """
        return ExpressionEvaluator.evaluate(self, obj)

    def executeInstruction(self, instruction):
        """
        Dispatch a single instruction to its registered handler.

        @param instruction: Instruction as a string (action name) or dict.
        @raise Exception: If instruction type is invalid.
        """
        if isinstance(instruction, str):
            return self.execute(instruction)

        if not isinstance(instruction, dict):
            raise Exception(f"Invalid instruction type: {type(instruction)}")

        name   = next(iter(instruction))
        params = instruction[name]

        if name in CONTROL_ACTIONS:
            return self.execute(name, params)

        return self.execute(name, ExpressionEvaluator.evaluate(self, params))

    def normalize(self, value):
        """
        Evaluate all expressions within a value.

        @param value: Value or nested structure.
        @return: Evaluated result.
        """
        return ExpressionEvaluator.evaluate(self, value)

    def resolveVariables(self, value):
        """
        Replace variable references (prefixed with '$') with their stored values.

        @param value: Value, list, or dict potentially containing variable references.
        @return: Value with all variable references replaced.
        @raise Exception: If a referenced variable is not defined.
        """
        if isinstance(value, str) and value.startswith("$"):
            variableName = value[1:]
            if variableName not in self.variable:
                raise Exception(f"Unknown variable '{variableName}'")
            return self.variable[variableName]

        if isinstance(value, list):
            return [self.resolveVariables(v) for v in value]

        if isinstance(value, dict):
            return {k: self.resolveVariables(v) for k, v in value.items()}

        return value

    def execute(self, name: str, params=None):
        """
        Look up and call a registered action handler.

        @param name: Action name.
        @type name: str
        @param params: Action parameters, or None for zero-argument actions.
        @return: Handler return value, or None on error.
        """
        handler = self.registry.get(name)
        if handler is None:
            cm.MKprintWarning(f"Unknown action: {name}", SCRIPT_NAME, SCRIPT_COLOR)
            return None

        try:
            if params is None:
                return handler(self)
            return handler(self, params)
        except Exception as e:
            cm.MKprintWarning(f"Error executing action '{name}': {e}", SCRIPT_NAME, SCRIPT_COLOR)

    def print(self, msg: str) -> None:
        """@param msg: Message to print at info level."""
        cm.MKprint(msg, SCRIPT_NAME, SCRIPT_COLOR)

    def printWarning(self, msg: str) -> None:
        """@param msg: Message to print at warning level."""
        cm.MKprintWarning(msg, SCRIPT_NAME, SCRIPT_COLOR)

    def printDebug(self, msg: str) -> None:
        """@param msg: Message to print at debug level."""
        cm.MKprintDebug(msg, SCRIPT_NAME, SCRIPT_COLOR)

    def printSafety(self, msg: str) -> None:
        """@param msg: Message to print at safety level."""
        cm.MKprintSafety(msg, SCRIPT_NAME, SCRIPT_COLOR)


class Validator:
    """
    Fluent parameter validator for action handlers.

    @ivar isValid: Whether all checks have passed so far.
    @ivar context: Action name for error messages.
    @ivar fields: The dict or value being validated.
    """

    def __init__(self, fields, context: str = "unknown"):
        """
        @param fields: Dict (or other value) to validate.
        @param context: Action name for error messages.
        @type context: str
        """
        self.isValid = True
        self.context = context
        self.fields  = fields

    def require(self, field: str) -> "Validator":
        """
        Assert that a required key is present and non-None.

        @param field: Key to check.
        @type field: str
        @rtype: Validator
        """
        if self.fields.get(field) is None:
            self.isValid = False
            cm.MKprintWarning(
                f"Missing required parameter '{field}' for action '{self.context}'.",
                SCRIPT_NAME, SCRIPT_COLOR
            )
        return self

    def isAList(self) -> "Validator":
        """
        Assert that the validated value is a list.

        @rtype: Validator
        """
        if not isinstance(self.fields, list):
            self.isValid = False
            cm.MKprintWarning(
                f"Parameters should be a list for action '{self.context}'.",
                SCRIPT_NAME, SCRIPT_COLOR
            )
        return self

    def validate(self) -> bool:
        """
        Return True if all checks passed.

        @rtype: bool
        """
        return self.isValid


class ExpressionEvaluator:
    """
    Recursive expression evaluator for YAML-embedded arithmetic and logic.

    Supports operators: add, sub, mul, div, mod, pow, abs, min, max, clamp,
    eq, neq, gt, gte, lt, lte, and, or, not, vec_add, vec_sub, distance,
    length, normalize.

    Variable references are dollar-prefixed strings (e.g. '$myVar').
    """

    KNOWN_OPERATORS = {
        "add", "sub", "mul", "div", "mod", "pow", "abs", "min", "max", "clamp",
        "eq", "neq", "gt", "gte", "lt", "lte",
        "and", "or", "not",
        "vec_add", "vec_sub", "distance", "length", "normalize",
    }

    @staticmethod
    def evaluate(executor: "Executor", value):
        """
        Recursively evaluate an expression.

        @param executor: Executor providing the variable store.
        @type executor: Executor
        @param value: Scalar, list, dict expression, or variable reference.
        @return: Evaluated result.
        """
        if value is None:
            return None

        if isinstance(value, (bool, int, float)):
            return value

        if isinstance(value, str):
            if value.startswith("$"):
                return executor.variable.get(value[1:])
            return value

        if isinstance(value, list):
            return [ExpressionEvaluator.evaluate(executor, v) for v in value]

        if not isinstance(value, dict):
            return value

        if len(value) != 1:
            operator = next(iter(value))
            if operator in ExpressionEvaluator.KNOWN_OPERATORS:
                operands = value[operator]
            return {k: ExpressionEvaluator.evaluate(executor, v) for k, v in value.items()}

        operator = next(iter(value))
        operands = value[operator]

        if operator == "add":
            values = [ExpressionEvaluator.evaluate(executor, v) for v in operands]
            if any(isinstance(v, str) for v in values):
                return "".join(str(v) for v in values)
            return sum(values)

        if operator == "sub":
            return (ExpressionEvaluator.evaluate(executor, operands[0])
                    - ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "mul":
            result = 1
            for operand in operands:
                result *= ExpressionEvaluator.evaluate(executor, operand)
            return result

        if operator == "div":
            return (ExpressionEvaluator.evaluate(executor, operands[0])
                    / ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "mod":
            return (ExpressionEvaluator.evaluate(executor, operands[0])
                    % ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "pow":
            return (ExpressionEvaluator.evaluate(executor, operands[0])
                    ** ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "abs":
            return abs(ExpressionEvaluator.evaluate(executor, operands))

        if operator == "min":
            return min(ExpressionEvaluator.evaluate(executor, v) for v in operands)

        if operator == "max":
            return max(ExpressionEvaluator.evaluate(executor, v) for v in operands)

        if operator == "clamp":
            value_ = ExpressionEvaluator.evaluate(executor, operands[0])
            min_   = ExpressionEvaluator.evaluate(executor, operands[1])
            max_   = ExpressionEvaluator.evaluate(executor, operands[2])
            return max(min_, min(value_, max_))

        if operator == "eq":
            return (ExpressionEvaluator.evaluate(executor, operands[0])
                    == ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "neq":
            return (ExpressionEvaluator.evaluate(executor, operands[0])
                    != ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "gt":
            return (ExpressionEvaluator.evaluate(executor, operands[0])
                    > ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "gte":
            return (ExpressionEvaluator.evaluate(executor, operands[0])
                    >= ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "lt":
            return (ExpressionEvaluator.evaluate(executor, operands[0])
                    < ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "lte":
            return (ExpressionEvaluator.evaluate(executor, operands[0])
                    <= ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "and":
            return all(ExpressionEvaluator.evaluate(executor, v) for v in operands)

        if operator == "or":
            return any(ExpressionEvaluator.evaluate(executor, v) for v in operands)

        if operator == "not":
            return not ExpressionEvaluator.evaluate(executor, operands)

        if operator == "vec_add":
            a = ExpressionEvaluator.evaluate(executor, operands[0])
            b = ExpressionEvaluator.evaluate(executor, operands[1])
            return [x + y for x, y in zip(a, b)]

        if operator == "vec_sub":
            a = ExpressionEvaluator.evaluate(executor, operands[0])
            b = ExpressionEvaluator.evaluate(executor, operands[1])
            return [x - y for x, y in zip(a, b)]

        if operator == "distance":
            a = ExpressionEvaluator.evaluate(executor, operands[0])
            b = ExpressionEvaluator.evaluate(executor, operands[1])
            return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

        if operator == "length":
            vec = ExpressionEvaluator.evaluate(executor, operands)
            return math.sqrt(sum(v * v for v in vec))

        if operator == "normalize":
            vec    = ExpressionEvaluator.evaluate(executor, operands)
            length = math.sqrt(sum(v * v for v in vec))
            if length < 1e-8:
                return vec
            return [v / length for v in vec]

        return {k: ExpressionEvaluator.evaluate(executor, v) for k, v in value.items()}
