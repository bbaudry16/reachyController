import time
import os

# inspired by Yoshman29's console manager: https://github.com/YoshiCrafter29


class Color:
    """ANSI color codes for terminal output."""
    RED            = "\x1b[1;31m"
    BLACK          = "\x1b[1;30m"
    GREEN          = "\x1b[1;32m"
    YELLOW         = "\x1b[1;33m"
    BLUE           = "\x1b[1;34m"
    MAGENTA        = "\x1b[1;35m"
    CYAN           = "\x1b[1;36m"
    WHITE          = "\x1b[1;37m"
    BRIGHT_RED     = "\x1b[1;91m"
    BRIGHT_BLACK   = "\x1b[1;90m"
    BRIGHT_GREEN   = "\x1b[1;92m"
    BRIGHT_YELLOW  = "\x1b[1;93m"
    BRIGHT_BLUE    = "\x1b[1;94m"
    BRIGHT_MAGENTA = "\x1b[1;95m"
    BRIGHT_CYAN    = "\x1b[1;96m"
    BRIGHT_WHITE   = "\x1b[1;97m"
    DEFAULT        = "\x1b[1;38m"
    BOLD           = "\033[1m"
    RESET          = "\x1b[1;39m"
    SAFETY         = RED
    DEBUG          = YELLOW
    WARNING        = YELLOW


MAX_SCRIPT_NAME_CHAR : int = 17

indentation     : int = 0
INDENTATION_STR : str = "   "
INDENTATION_INDICATOR : str = "⌞"


def addIndentation(n: int = 1) -> None:
    """
    Increase the current indentation level.

    @param n: Number of levels to add (ignored if negative).
    @type n: int
    """
    global indentation
    if n < 0:
        return
    indentation += n


def removeIndentation(n: int = 1) -> None:
    """
    Decrease the current indentation level, clamped to zero.

    @param n: Number of levels to remove.
    @type n: int
    """
    global indentation
    indentation -= n
    indentation = max(0, indentation)


# Legacy aliases kept for backward compatibility
def addIntentation(n: int = 1) -> None:
    """@deprecated: Use L{addIndentation} instead."""
    addIndentation(n)


def removeIntentation(n: int = 1) -> None:
    """@deprecated: Use L{removeIndentation} instead."""
    removeIndentation(n)


def MKprint(printStr: str, instrName: str = "default", colorID: "Color" = Color.DEFAULT) -> None:
    """
    Print a timestamped, indented, colored message to stdout.

    @param printStr: Message body.
    @type printStr: str
    @param instrName: Source module name (truncated to L{MAX_SCRIPT_NAME_CHAR}).
    @type instrName: str
    @param colorID: ANSI color code for the header.
    @type colorID: str
    """
    global indentation
    name = " " * max(0, (MAX_SCRIPT_NAME_CHAR - len(instrName))) + instrName
    name = name[:MAX_SCRIPT_NAME_CHAR]
    header = coloredStr("[" + getHourStr() + " - " + name + "] ", colorID)
    if indentation == 0:
        print(header + printStr + "\n", end="")
    else:
        prefix = INDENTATION_STR * indentation + INDENTATION_INDICATOR + " "
        print(header + prefix + printStr + "\n", end="")


def MKprintSafety(printStr: str, instrName: str = "default", colorID: "Color" = Color.DEFAULT) -> None:
    """
    Print a safety-level message (prefixed with [SAFETY]).

    @param printStr: Message body.
    @type printStr: str
    @param instrName: Source module name.
    @type instrName: str
    @param colorID: ANSI color code.
    @type colorID: str
    """
    MKprint(Color.SAFETY + "[SAFETY] " + printStr + Color.RESET, instrName, colorID)


def MKprintDebug(printStr: str, instrName: str = "default", colorID: "Color" = Color.DEFAULT) -> None:
    """
    Print a debug-level message (prefixed with [DEBUG]).

    @param printStr: Message body.
    @type printStr: str
    @param instrName: Source module name.
    @type instrName: str
    @param colorID: ANSI color code.
    @type colorID: str
    """
    MKprint(Color.DEBUG + "[DEBUG] " + printStr + Color.RESET, instrName, colorID)


def MKprintWarning(printStr: str, instrName: str = "default", colorID: "Color" = Color.DEFAULT) -> None:
    """
    Print a warning-level message (prefixed with [WARNING]).

    @param printStr: Message body.
    @type printStr: str
    @param instrName: Source module name.
    @type instrName: str
    @param colorID: ANSI color code.
    @type colorID: str
    """
    MKprint(Color.WARNING + "[WARNING] " + printStr + Color.RESET, instrName, colorID)


def getHourStr() -> str:
    """
    Return the current time as HH:MM:SS.

    @rtype: str
    """
    return time.strftime("%H:%M:%S")


def coloredStr(string: str, colorID: str) -> str:
    """
    Wrap a string with an ANSI color code and a reset suffix.

    @param string: Text to colorize.
    @type string: str
    @param colorID: ANSI color escape code.
    @type colorID: str
    @rtype: str
    """
    return colorID + string + Color.RESET


def testColor() -> None:
    """Print a sample line for every defined color in L{Color}."""
    for name, value in Color.__dict__.items():
        if name.startswith("__") or callable(value):
            continue
        try:
            MKprint(name + " = TEST", name, value)
        except Exception:
            pass


os.system('')
