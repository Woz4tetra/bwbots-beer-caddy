import copy

class JoystickMessage:
    def __init__(self, num_buttons=0, num_axes=0) -> None:
        self.buttons = [False for _ in range(num_buttons)]
        self.axes = [0.0 for _ in range(num_axes)]
    
    @classmethod
    def from_msg(cls, other: "JoystickMessage"):
        self = cls()
        if isinstance(other, JoystickMessage):
            self.buttons = copy.copy(other.buttons)
            self.axes = copy.copy(other.axes)
        else:
            raise ValueError(f"Can't copy non Joystick message: {repr(other)}")
        return self
    
    def set_num_buttons(self, num_buttons):
        self.buttons = [False for _ in range(num_buttons)]
    
    def set_num_axes(self, num_axes):
        self.axes = [0.0 for _ in range(num_axes)]

    def __eq__(self, other: object) -> bool:
        if isinstance(other, JoystickMessage):
            return self.buttons == other.buttons and self.axes == other.axes
        else:
            return False

    def __nonzero__(self):
        return not (len(self.buttons) == 0 and len(self.axes) == 0)
    
    __bool__ = __nonzero__

    def __str__(self) -> str:
        return f"{self.__class__.__name__}: <{self.axes}, {self.buttons}>"
    
    __repr__ = __str__
