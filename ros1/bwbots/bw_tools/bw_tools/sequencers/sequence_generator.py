from enum import IntEnum
from bw_interfaces.msg import BwSequence
from bw_interfaces.msg import BwSequenceElement


class BwSequenceType(IntEnum):
    START_TONE = 0
    STOP_TONE = 1
    DELAY = 2
    SET_RING_LED = 3
    SHOW_LED = 4

def clamp(value, lower, upper):
    return min(upper, max(value, lower))


class SequenceGenerator:
    SERIAL_COUNTER = 0
    def __init__(self) -> None:
        self.serial = SequenceGenerator.SERIAL_COUNTER
        SequenceGenerator.SERIAL_COUNTER += 1
        if SequenceGenerator.SERIAL_COUNTER >= 8:
            raise ValueError("Can't have more than 8 sequences")
        self.msg = BwSequence()
        self.msg.serial = self.serial
    
    def add(self, *elements):
        for element in elements:
            self.msg.sequence.append(element)
        
    def iter(self):
        for element in self.msg.sequence:
            yield element

    def __len__(self):
        return len(self.msg.sequence)

    @classmethod
    def make_start_tone(cls, channel: int, frequency: int, volume: int):
        tone_msg = BwSequenceElement()
        channel = clamp(int(channel), 0, 4)
        frequency = clamp(int(frequency), 0, 0xffff)
        volume = clamp(int(volume), 0, 0xff)
        tone_msg.parameters = (channel << 28) | (volume << 20) | (frequency << 4) | BwSequenceType.START_TONE
        return tone_msg

    @classmethod
    def make_stop_tone(cls, channel: int):
        tone_msg = BwSequenceElement()
        channel = clamp(int(channel), 0, 4)
        tone_msg.parameters = (channel << 28) | BwSequenceType.STOP_TONE
        return tone_msg

    @classmethod
    def make_delay(cls, delay_ms):
        delay_msg = BwSequenceElement()
        delay_ms = clamp(int(delay_ms), 0, 100000)
        delay_msg.parameters = (delay_ms << 4) | BwSequenceType.DELAY
        return delay_msg

    @classmethod
    def make_tone(cls, channel, frequency, volume, duration_ms):
        tone_msg = cls.make_start_tone(channel, frequency, volume)
        delay_msg = cls.make_delay(duration_ms)
        return tone_msg, delay_msg

    @classmethod
    def make_led(cls, index, r, g, b, w):
        led_msg = BwSequenceElement()
        index = clamp(int(index), 0, 0xffff)
        r = clamp(int(r), 0, 0xff)
        g = clamp(int(g), 0, 0xff)
        b = clamp(int(b), 0, 0xff)
        w = clamp(int(w), 0, 0xff)
        led_msg.parameters = (index << 36) | (w << 28) | (r << 20) | (g << 12) | (b << 4) | BwSequenceType.SET_RING_LED
        return led_msg

    @classmethod
    def make_show(cls):
        show_msg = BwSequenceElement()
        show_msg.parameters = BwSequenceType.SHOW_LED
        return show_msg
