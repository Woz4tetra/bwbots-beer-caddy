from bw_tools.enum_auto_lower import EnumAutoLowerStr, auto


class Mode(EnumAutoLowerStr):
    UNDOCK = auto()
    DOCK = auto()
    DELIVER = auto()
    IDLE = auto()
