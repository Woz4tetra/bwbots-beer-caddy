# adapted from https://gist.github.com/rdb/8864666

import time
import array
import queue
import struct
import select
import asyncio
from fcntl import ioctl
from .joystick import Joystick
from .joystick_message import JoystickMessage
from lib.recursive_namespace import RecursiveNamespace


AXES = [
    0x00,  # x
    0x01,  # y
    0x02,  # z
    0x03,  # rx
    0x04,  # ry
    0x05,  # rz
    0x06,  # throttle
    0x07,  # rudder
    0x08,  # wheel
    0x09,  # gas
    0x0a,  # brake
    0x10,  # hat0x
    0x11,  # hat0y
    0x12,  # hat1x
    0x13,  # hat1y
    0x14,  # hat2x
    0x15,  # hat2y
    0x16,  # hat3x
    0x17,  # hat3y
    0x18,  # pressure
    0x19,  # distance
    0x1a,  # tilt_x
    0x1b,  # tilt_y
    0x1c,  # tool_width
    0x20,  # volume
    0x28,  # misc
]

AXES_MAP = {value: index for index, value in enumerate(AXES)}

BUTTONS = [
    0x120,  # trigger
    0x121,  # thumb
    0x122,  # thumb2
    0x123,  # top
    0x124,  # top2
    0x125,  # pinkie
    0x126,  # base
    0x127,  # base2
    0x128,  # base3
    0x129,  # base4
    0x12a,  # base5
    0x12b,  # base6
    0x12f,  # dead
    0x130,  # a
    0x131,  # b
    0x132,  # c
    0x133,  # x
    0x134,  # y
    0x135,  # z
    0x136,  # tl
    0x137,  # tr
    0x138,  # tl2
    0x139,  # tr2
    0x13a,  # select
    0x13b,  # start
    0x13c,  # mode
    0x13d,  # thumbl
    0x13e,  # thumbr

    0x220,  # dpad_up
    0x221,  # dpad_down
    0x222,  # dpad_left
    0x223,  # dpad_right

    # XBox 360 controller uses these codes.
    0x2c0, # dpad_left
    0x2c1, # dpad_right
    0x2c2, # dpad_up
    0x2c3, # dpad_down
]

BUTTONS_MAP = {value: index for index, value in enumerate(BUTTONS)}


class UnixJoystick(Joystick):
    def __init__(self, 
            logger, address,
            button_mapping: RecursiveNamespace = RecursiveNamespace(),
            axis_mapping: RecursiveNamespace = RecursiveNamespace()):
        self.logger = logger

        self.jsdev = None
        self.address = address
        self.prev_open_attempt_time = time.time()

        self.prev_load_time = time.time()
        self.config_load_interval = 1.0

        self.evbuf_queue = queue.Queue()
        self.buffered_message = JoystickMessage()

        this_axis_mapping = RecursiveNamespace(**{
            "left": {
                "x": 0,  # x
                "y": 1,  # y
                "z": 2,  # z
            },
            "right": {
                "x": 3,  # rx
                "y": 4,  # ry
                "z": 5,  # rz
            },
            "brake": {
                "left": 6,  # throttle
                "right": 7,  # rudder
            },
            "pedals": {
                "wheel": 8,  # wheel
                "gas": 9,  # gas
                "brake": 10,  # brake
            },
            "dpad0": {
                "x": 11,  # hat0x
                "y": 12,  # hat0y
            },
            "dpad1": {
                "x": 13,  # hat1x
                "y": 14,  # hat1y
            },
            "dpad2": {
                "x": 15,  # hat2x
                "y": 16,  # hat2y
            },
            "dpad3": {
                "x": 17,  # hat3x
                "y": 18,  # hat3y
            },
            "pressure": 19,  # pressure
            "distance": 20,  # distance
            "tilt_x": 21,  # tilt_x
            "tilt_y": 22,  # tilt_y
            "tool_width": 23,  # tool_width
            "volume": 24,  # volume
            "misc": 25,  # misc

        })
        this_button_mapping = RecursiveNamespace(**{
            "trigger": 0,
            "thumb": {
                "1": 1,  # thumb
                "2": 2,  # thumb2
            },
            "top": {
                "1": 3,  # top
                "2": 4,  # top2
            },
            "pinkie": 5,
            "base": {
                "1": 6,  # base
                "2": 7,  # base2
                "3": 8,  # base3
                "4": 9,  # base4
                "5": 10,  # base5
                "6": 11,  # base6
            },
            "dead": 12,
            "main": {
                "a": 13,
                "b": 14,
                "c": 15,
                "x": 16,
                "y": 17,
                "z": 18,
            },
            "trigger1": {
                "left": 19,  # tl
                "right": 20,  # tr
            },
            "trigger2": {
                "left": 21,  # tl2
                "right": 22,  # tr2
            },
            "menu": {
                "select": 23,  # select
                "start": 24,  # start
                "mode": 25,  # mode
            },
            "thumb_stick": {
                "left": 26,  # thumbl
                "right": 27,  # thumbr
            },
            "dpad": {
                "up": 28,  # dpad_up
                "down": 29, # dpad_down
                "left": 30, # dpad_left
                "right": 31, # dpad_right
            }
        })
        this_button_mapping.merge(button_mapping)
        this_axis_mapping.merge(axis_mapping)

        super().__init__(this_button_mapping, this_axis_mapping)

    def start(self):
        self.prev_open_attempt_time = time.time()

        self.open_joystick()

    def stop(self):
        self.close_joystick()

    def is_open(self):
        return self.jsdev is not None

    def open_joystick(self):
        try:
            self.jsdev = open(self.address, 'rb')

            self.logger.info("Joystick: %s" % self.get_device_name())
            self.get_axes_buttons()
            # self.get_axis_map()
            # self.get_button_map()

            self.logger.info(f"{len(self.buffered_message.axes)} axes found")
            self.logger.info(f"{len(self.buffered_message.buttons)} buttons found")
        except FileNotFoundError:
            pass
        except BaseException as e:
            self.logger.error(str(e), exc_info=True)

    def close_joystick(self):
        self.logger.info("Closing joystick")
        self.jsdev.close()
        self.jsdev = None

    def get_device_name(self):
        buf = array.array('B', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
        return js_name

    def get_axes_buttons(self):
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]
        self.buffered_message.set_num_axes(num_axes)

        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]
        self.buffered_message.set_num_buttons(num_buttons)

    # def get_axis_map(self):
    #     buf = array.array('B', [0] * 0x40)
    #     ioctl(self.jsdev, 0x80406a32, buf) # JSIOCGAXMAP
    #     for axis in buf[:self.num_axes]:
    #         axis_num = AXES_MAP.get(axis, 'unknown(0x%02x)' % axis)

    # def get_button_map(self):
    #     buf = array.array('H', [0] * 200)
    #     ioctl(self.jsdev, 0x80406a34, buf) # JSIOCGBTNMAP
    #     for btn in buf[:self.num_buttons]:
    #         btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    #         self.button_map.append(btn_name)
    #         self.button_states[btn_name] = 0

    async def update(self):
        await self.check_joystick_events()
        self.set_msg(self.buffered_message)
        await asyncio.sleep(0.0)

    async def check_joystick_events(self):
        if not self.is_open():
            self.open_joystick()
            await asyncio.sleep(1.0)
            if self.jsdev:
                self.logger.info("Joystick opened with address {}".format(self.address))
            return

        try:
            while self.jsdev in select.select([self.jsdev], [], [], 0)[0]:
                evbuf = self.jsdev.read(8)
                self.parse_joystick_bytes(evbuf)
        except OSError:
            self.close_joystick()
        except BaseException as e:
            self.logger.error(str(e), exc_info=True)
            self.close_joystick()

    def parse_joystick_bytes(self, evbuf):
        evtime, value, type, number = struct.unpack('IhBB', evbuf)

        if type & 0x01:
            if number in BUTTONS_MAP:
                button_index = BUTTONS_MAP[number]
                self.buffered_message.buttons[button_index] = bool(value)

        if type & 0x02:
            if number in AXES_MAP:
                axis_index = AXES_MAP[number]
                self.buffered_message.axes[axis_index] = value / 32767.0
