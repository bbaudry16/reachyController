import yaml as yml
from . import consoleManager as cm
from . import reachyController
from .actionRegistry import ACTION_REGISTRY 
        
class Instructor:

    SCRIPT_NAME : str = "Instructor"
    SCRIPT_COLOR : str = cm.Color.BRIGHT_MAGENTA

    def __init__(self, data : dict, reachyController : "reachyController.ReachyController"):
        self.data = data
        self.executor = Executor(reachyController)

    def execute(self):
        for instruction in self.data:
            self.executeInstruction(instruction)

    def executeInstruction(self, instruction):

        if isinstance(instruction, str):
            self.executor.execute(instruction)

        elif isinstance(instruction, dict):
            name = next(iter(instruction))
            params = instruction[name]
            self.executor.execute(name, params)
    
    @classmethod
    def loadFromPath(self, path : str, reachyController : "reachyController.ReachyController") -> "Instructor":
        try :
            stream = open(path, 'r')
            load = yml.load(stream)
            cm.MKprint("successfully loaded : " + path, Instructor.SCRIPT_NAME, Instructor.SCRIPT_COLOR)
            return Instructor(load, reachyController) 

        except Exception as e:
            cm.MKprintWarning("Failed to load : " + path + " , returning Parser({}) with error : " + str(e), Instructor.SCRIPT_NAME, Instructor.SCRIPT_COLOR)
            return Instructor({}, reachyController)

class Executor:

    def __init__(self, reachy):
        self.reachy = reachy
        self.registry = ACTION_REGISTRY

    def execute(self, name : str, params=None):
        
        handler = self.registry.get(name)
        if handler == None:
            cm.MKprintWarning("unknown action : " + name, Instructor.SCRIPT_NAME, Instructor.SCRIPT_COLOR)
            return None

        if params is None:
            return handler(self)

        return handler(self, params)
        