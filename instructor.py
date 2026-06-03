import yaml as yml
from . import consoleManager as cm
from . import reachyController
from .actionRegistry import ACTION_REGISTRY 
        
SCRIPT_NAME : str = "Instructor"
SCRIPT_COLOR : str = cm.Color.BRIGHT_MAGENTA

class Instructor:


    def __init__(self, data : dict, reachyController : "reachyController.ReachyController"):
        self.data = data
        print(data)
        self.executor = Executor(reachyController)

    def execute(self):
        for instruction in self.data:
            self.executor.executeInstruction(instruction)
    
    @classmethod
    def loadFromPath(self, path : str, reachyController : "reachyController.ReachyController") -> "Instructor":
        try :
            stream = open(path, 'r')
            load = yml.load(stream)
            cm.MKprint("successfully loaded : " + path, SCRIPT_NAME, SCRIPT_COLOR)
            return Instructor(load, reachyController) 

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
            self.execute(instruction)

        elif isinstance(instruction, dict):
            name = next(iter(instruction))
            params = instruction[name]
            self.execute(name, params)

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
    
    