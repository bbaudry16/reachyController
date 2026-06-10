import yaml as yml
from . import consoleManager as cm
from . import reachyController
from .actionRegistry import ACTION_REGISTRY, CONTROL_ACTIONS
import math

SCRIPT_NAME : str = "Instructor"
SCRIPT_COLOR : str = cm.Color.BLUE

class Instructor:



    def __init__(self, data : dict, reachyController : "reachyController.ReachyController"):
        self.data = data
        self.executor = Executor(reachyController)

    def execute(self):
        cm.MKprint("executing instruction ┐", SCRIPT_NAME, SCRIPT_COLOR)
        cm.addIntentation()
        for instruction in self.data:
            self.executor.executeInstruction(instruction)
        cm.removeIntentation()
        cm.MKprint("done executing ┘", SCRIPT_NAME, SCRIPT_COLOR)
        
    
    @classmethod
    def loadFromPath(self, path : str, reachyController : "reachyController.ReachyController") -> "Instructor":
        try :
            stream = open(path, 'r')
            load = yml.safe_load(stream)
            if not isinstance(load, list) and Validator(load).require("format").require("reachy").validate() and load.get("format") == "reachy_instruction" and isinstance(load.get("reachy"), list):
                cm.MKprint("successfully loaded reachy instruction format (ryi) : " + path, SCRIPT_NAME, SCRIPT_COLOR)
                return Instructor(load.get("reachy"), reachyController)
            else:
                try:
                    cm.MKprint("successfully loaded yml instruction format : " + path, SCRIPT_NAME, SCRIPT_COLOR)
                    return Instructor(load, reachyController)
                except:
                    raise("invalid file format, check if it's a yml or a ryi, check if the file format is correct")

        except Exception as e:
            cm.MKprintWarning("Failed to load : " + path + " , returning Parser({}) with error : " + str(e), SCRIPT_NAME, SCRIPT_COLOR)
            return Instructor({}, reachyController)

    @classmethod
    def loadFromString(cls, yamlString: str, reachyController: "reachyController.ReachyController") -> "Instructor":
        try:
            # Correction automatique si reachy: manque
            stripped = yamlString.strip()
            if not stripped.startswith("reachy:"):
                yamlString = "reachy:\n" + "\n".join("- " + line if line and not line.startswith("-") and not line.startswith(" ") else line for line in stripped.splitlines())

            load = yml.safe_load(yamlString)
            if isinstance(load, dict) and "reachy" in load and isinstance(load.get("reachy"), list):
                cm.MKprint("successfully loaded ryi format from string", SCRIPT_NAME, SCRIPT_COLOR)
                return cls(load.get("reachy"), reachyController)
            elif isinstance(load, list):
                cm.MKprint("successfully loaded yml format from string", SCRIPT_NAME, SCRIPT_COLOR)
                return cls(load, reachyController)
            else:
                cm.MKprintWarning("Unknown format", SCRIPT_NAME, SCRIPT_COLOR)
                return cls([], reachyController)
        except Exception as e:
            cm.MKprintWarning("Failed to load from string : " + str(e), SCRIPT_NAME, SCRIPT_COLOR)
            return cls([], reachyController)

class Executor:

    def __init__(self, reachy : "reachyController.ReachyController"):
        self.reachy : "reachyController.ReachyController" = reachy
        self.registry : dict = ACTION_REGISTRY
        self.variable : dict = {}

    def resolveExpressions(self, obj):
        return ExpressionEvaluator.evaluate(self ,obj)

    def executeInstruction(self, instruction):

            if isinstance(instruction, str):
                return self.execute(instruction)

            if not isinstance(instruction, dict):
                raise Exception(f"Invalid instruction type: {type(instruction)}")

            name = next(iter(instruction))
            params = instruction[name]

            if name in CONTROL_ACTIONS:
                return self.execute(name, params)

            resolved_params = ExpressionEvaluator.evaluate(self, params)

            return self.execute(name, resolved_params)
            
    def normalize(self, value):
        return ExpressionEvaluator.evaluate(self, value)

    def resolveVariables(self, value):

        if isinstance(value, str) and value.startswith("$"):
            variableName = value[1:]

            if variableName not in self.variable:
                raise Exception(f"Unknown variable '{variableName}'")

            return self.variable[variableName]

        if isinstance(value, list):
            return [self.resolveVariables(v) for v in value]

        if isinstance(value, dict):
            return {
                k: self.resolveVariables(v)
                for k, v in value.items()
            }

        return value

    def execute(self, name : str, params=None):
        
        handler = self.registry.get(name)
        if handler == None:
            cm.MKprintWarning("unknown action : " + name, SCRIPT_NAME, SCRIPT_COLOR)
            return None
        
        if params is None:
            try:
                return handler(self)
            except Exception as e:
                cm.MKprintWarning("Error when executing action " + name + " : " + str(e), SCRIPT_NAME, SCRIPT_COLOR)

        try:
            return handler(self, params)
        except Exception as e:
            cm.MKprintWarning("Error when executing action " + name + " : " + str(e), SCRIPT_NAME, SCRIPT_COLOR)

    def print(self, msg : str):
        cm.MKprint(msg, SCRIPT_NAME, SCRIPT_COLOR)
    
    def printWarning(self, msg : str):
        cm.MKprintWarning(msg, SCRIPT_NAME, SCRIPT_COLOR)
    
    def printDebug(self, msg : str):
        cm.MKprintDebug(msg, SCRIPT_NAME, SCRIPT_COLOR)
    
    def printSafety(self, msg : str):
        cm.MKprintSafety(msg, SCRIPT_NAME, SCRIPT_COLOR)

class Validator():

    def __init__(self, fields,  context : str = "unknown"):
        self.isValid = True
        self.context = context
        self.fields = fields

    def require(self, field : str) -> "Validator":
        if self.fields.get(field) == None:
            self.isValid = False
            cm.MKprintWarning("missing required parameter : " + field + ", for action : " + self.context, SCRIPT_NAME, SCRIPT_COLOR)
        return self
    
    def isAList(self) -> "Validator":
        if not isinstance(self.fields, list):
            self.isValid = False
            cm.MKprintWarning("parameters should be a list, for action : " + self.context, SCRIPT_NAME, SCRIPT_COLOR)
        return self

    def validate(self) -> bool:
        return self.isValid
    
    
class ExpressionEvaluator:

    KNOWN_OPERATORS = {"add", "sub", "mul", "div", "mod", "pow","abs", "min", "max", "clamp","eq", "neq", "gt", "gte", "lt", "lte","and", "or", "not","vec_add", "vec_sub","distance", "length", "normalize"}

    @staticmethod
    def evaluate(executor : "Executor", value):

        if value is None:
            return None

        if isinstance(value, (bool, int, float)):
            return value

        if isinstance(value, str):

            if value.startswith("$"):
                return executor.variable.get(value[1:])

            return value

        if isinstance(value, list):
            return [ ExpressionEvaluator.evaluate(executor, v) for v in value]

        if not isinstance(value, dict):
            return value

        if len(value) != 1:
            operator = next(iter(value))

            if operator in ExpressionEvaluator.KNOWN_OPERATORS:

                operands = value[operator]
            return { k: ExpressionEvaluator.evaluate(executor, v) for k, v in value.items()}

        operator = next(iter(value))
        operands = value[operator]

        if operator == "add":
            values = [ExpressionEvaluator.evaluate(executor, v) for v in operands]

            if any(isinstance(v, str) for v in values):
                return "".join(str(v) for v in values)

            return sum(values)

        if operator == "sub":
            left = ExpressionEvaluator.evaluate(executor, operands[0])
            right = ExpressionEvaluator.evaluate(executor, operands[1])
            return left - right

        if operator == "mul":
            result = 1

            for operand in operands:
                result *= ExpressionEvaluator.evaluate(executor, operand)

            return result

        if operator == "div":
            left = ExpressionEvaluator.evaluate(executor, operands[0])
            right = ExpressionEvaluator.evaluate(executor, operands[1])
            return left / right

        if operator == "mod":
            left = ExpressionEvaluator.evaluate(executor, operands[0])
            right = ExpressionEvaluator.evaluate(executor, operands[1])
            return left % right

        if operator == "pow":
            left = ExpressionEvaluator.evaluate(executor, operands[0])
            right = ExpressionEvaluator.evaluate(executor, operands[1])
            return left ** right

        if operator == "abs":
            return abs(ExpressionEvaluator.evaluate(executor, operands))

        if operator == "min":
            return min(ExpressionEvaluator.evaluate(executor, v) for v in operands)

        if operator == "max":
            return max(ExpressionEvaluator.evaluate(executor, v) for v in operands)

        if operator == "clamp":
            value_ = ExpressionEvaluator.evaluate(executor, operands[0])
            min_ = ExpressionEvaluator.evaluate(executor, operands[1])
            max_ = ExpressionEvaluator.evaluate(executor, operands[2])

            return max(min_, min(value_, max_))

        if operator == "eq":
            return (ExpressionEvaluator.evaluate(executor, operands[0]) == ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "neq":
            return (ExpressionEvaluator.evaluate(executor, operands[0]) != ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "gt":
            return (ExpressionEvaluator.evaluate(executor, operands[0]) > ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "gte":
            return (ExpressionEvaluator.evaluate(executor, operands[0]) >= ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "lt":
            return (ExpressionEvaluator.evaluate(executor, operands[0]) < ExpressionEvaluator.evaluate(executor, operands[1]))

        if operator == "lte":
            return (ExpressionEvaluator.evaluate(executor, operands[0]) <= ExpressionEvaluator.evaluate(executor, operands[1]))

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

            vec = ExpressionEvaluator.evaluate(executor, operands)

            length = math.sqrt(sum(v * v for v in vec))

            if length < 1e-8:
                return vec

            return [v / length for v in vec]

        return {k: ExpressionEvaluator.evaluate(executor, v) for k, v in value.items()}