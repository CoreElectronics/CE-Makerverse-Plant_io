"""
ansi_colours.py

Exports a class for storing ANSI colour codes
and a method for using these codes to print in
colour or with decoration
"""


class Colours:
    """ANSI colour codes"""

    def __init__(self):
        pass

    def __str__(self):
        return str.__str__(self)

    BLACK = "\033[0;30m"
    LIGHT_RED = "\033[0;31m"
    LIGHT_GREEN = "\033[0;32m"
    BROWN = "\033[0;33m"
    LIGHT_BLUE = "\033[0;34m"
    PURPLE = "\033[0;35m"
    CYAN = "\033[0;36m"
    LIGHT_GRAY = "\033[0;37m"
    DARK_GRAY = "\033[1;30m"
    RED = "\033[1;31m"
    GREEN = "\033[1;32m"
    YELLOW = "\033[1;33m"
    BLUE = "\033[1;34m"
    LIGHT_PURPLE = "\033[1;35m"
    LIGHT_CYAN = "\033[1;36m"
    LIGHT_WHITE = "\033[1;37m"
    BOLD = "\033[1m"
    FAINT = "\033[2m"
    ITALIC = "\033[3m"
    UNDERLINE = "\033[4m"
    BLINK = "\033[5m"
    NEGATIVE = "\033[7m"
    CROSSED = "\033[9m"
    END = "\033[0m"


def colour_print(val, *colours):
    """
    Wraps print with ANSI color codes starting
    with the specified color (default of black)

    e.g. colour_print("Hello World", Colours.UNDERLINE, COLOURS.BOLD, Colours.RED)
    """
    print(f"{''.join(colours)}{str(val)}{Colours.END}")
