import math
import warnings
from sensor_msgs.msg import Joy
from bw_tools.recursive_namespace import RecursiveNamespace


class Joystick:
    MAX_JOY_VAL = 1.0
    
    def __init__(self, button_mapping, axis_mapping):
        self.prev_msg = None
        self.curr_msg = None

        self.button_mapping = RecursiveNamespace(**button_mapping)
        self.axis_mapping = RecursiveNamespace(**axis_mapping)

    def update(self, msg):
        if self.prev_msg is None:
            self.prev_msg = msg
        else:
            self.prev_msg = self.curr_msg
        self.curr_msg = msg

    def get_index(self, namespace, value):
        if isinstance(value, int):
            index = value
        elif isinstance(value, str):
            index = namespace.get_nested(value.split("/"))
        elif isinstance(value, tuple) or isinstance(value, list):
            index = namespace.get_nested(value)
        else:
            raise ValueError("Invalid index value type: %s<%s>" % (type(value), repr(value)))
        return index
    
    def is_valid_button(self, index):
        if index >= len(self.curr_msg.buttons):
            warnings.warn("Button '%s' does not map to a valid index: %s" % (index, self.curr_msg))
            return False
        else:
            return True
    
    def is_valid_axis(self, index):
        if index >= len(self.curr_msg.axes):
            warnings.warn("Axis '%s' does not map to a valid index: %s" % (index, self.curr_msg))
            return False
        else:
            return True
        
    def is_button_down(self, button):
        index = self.get_index(self.button_mapping, button)
        if not self.is_valid_button(index):
            raise ValueError("Invalid button: %s" % button)
        return self.curr_msg.buttons[index]

    def is_button_up(self, button):
        index = self.get_index(self.button_mapping, button)
        if not self.is_valid_button(index):
            raise ValueError("Invalid button: %s" % button)
        return not self.curr_msg.buttons[index]
    
    def did_button_down(self, button):
        return self.is_button_down(button) and self.did_button_change(button)

    def did_button_up(self, button):
        return self.is_button_up(button) and self.did_button_change(button)

    def did_button_change(self, button):
        index = self.get_index(self.button_mapping, button)
        if not self.is_valid_button(index):
            raise ValueError("Invalid button: %s" % button)
        return self.prev_msg.buttons[index] != self.curr_msg.buttons[index]
    
    def did_axis_change(self, axis):
        index = self.get_index(self.axis_mapping, axis)
        if not self.is_valid_axis(index):
            raise ValueError("Invalid axis: %s" % axis)
        return self.prev_msg.axes[index] != self.curr_msg.axes[index]
    
    def did_axis_less_than(self, axis, threshold):
        index = self.get_index(self.axis_mapping, axis)
        if not self.is_valid_axis(index):
            raise ValueError("Invalid axis: %s" % axis)
        return self.curr_msg.axes[index] < threshold and self.prev_msg.axes[index] >= threshold
    
    def did_axis_greater_than(self, axis, threshold):
        index = self.get_index(self.axis_mapping, axis)
        if not self.is_valid_axis(index):
            raise ValueError("Invalid axis: %s" % axis)
        return self.curr_msg.axes[index] > threshold and self.prev_msg.axes[index] <= threshold
    
    def check_list(self, fn, *keys):
        results = []
        for key in keys:
            results.append(fn(key))
        if len(results) == 1:
            return results[0]
        else:
            return results
    
    def get_axis(self, axis):
        index = self.get_index(self.axis_mapping, axis)
        if not self.is_valid_axis(index):
            raise ValueError("Invalid axis: %s" % axis)
        value = self.curr_msg.axes[index]
        return value

    def deadband_axis(self, axis, deadband, scale=1.0):
        value = self.get_axis(axis)
        if value is None:
            return None
        if abs(value) < deadband:
            return 0.0
        joy_val = abs(value) - deadband
        joy_val = math.copysign(joy_val, value)
        max_joy_val_adj = self.MAX_JOY_VAL - deadband
        command = scale / max_joy_val_adj * joy_val

        return command

