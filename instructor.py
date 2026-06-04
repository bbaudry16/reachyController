import yaml as yml
from . import consoleManager as cm
from . import reachyController
from .actionRegistry import ACTION_REGISTRY, CONTROL_ACTIONS
        
SCRIPT_NAME : str = "Instructor"
SCRIPT_COLOR : str = cm.Color.BLUE

class Instructor:


    def __init__(self, data : dict, reachyController : "reachyController.ReachyController"):
        self.data = data
        self.executor = Executor(reachyController)

    def execute(self):
        for instruction in self.data:
            self.executor.executeInstruction(instruction)
    
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

class Executor:

    def __init__(self, reachy : "reachyController.ReachyController"):
        self.reachy : "reachyController.ReachyController" = reachy
        self.registry : dict = ACTION_REGISTRY
        self.variable : dict = {}

    def executeInstruction(self, instruction):

        if isinstance(instruction, str):
            return self.execute(instruction)

        elif isinstance(instruction, dict):
            name = next(iter(instruction))
            params = instruction[name]

            if name not in CONTROL_ACTIONS:
                params = self.resolveVariables(params)
            return self.execute(name, params)
        
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
    
    