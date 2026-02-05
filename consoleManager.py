import time, os

#code from YoshiCr

class Color():
    RED = "\x1b[1;31m"
    BLACK = "\x1b[1;30m"
    GREEN = "\x1b[1;32m"
    YELLOW = "\x1b[1;33m"
    BLUE = "\x1b[1;34m"
    MAGENTA = "\x1b[1;35m"
    CYAN = "\x1b[1;36m"
    WHITE = "\x1b[1;37m",
    BRIGHT_RED = "\x1b[1;91m"
    BRIGHT_BLACK = "\x1b[1;90m"
    BRIGHT_GREEN = "\x1b[1;92m"
    BRIGHT_YELLOW = "\x1b[1;93m"
    BRIGHT_BLUE = "\x1b[1;94m"
    BRIGHT_MAGENTA = "\x1b[1;95m"
    BRIGHT_CYAN = "\x1b[1;96m"
    BRIGHT_WHITE = "\x1b[1;97m"
    DEFAULT = "\x1b[1;38m"
    BOLD = "\033[1m"
    RESET = "\x1b[1;39m"

def MKprint(printStr : str, instrName : str = "default", colorID : "Color" = Color.DEFAULT) -> None:
    print(coloredStr("[" + getHourStr() + " - "+ instrName + "] ", colorID)  + printStr)
    
def getHourStr() -> str:
    return time.strftime("%H:%M:%S")

def coloredStr(string : str, colorID : str) -> str:
    return colorID + string + Color.RESET

os.system('')