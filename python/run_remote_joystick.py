import os
import asyncio
import argparse
import subprocess

from lib.constants import *
from lib.exceptions import *
from lib.recursive_namespace import RecursiveNamespace
from lib.logger_manager import LoggerManager
from lib.session import Session
from lib.config import Config
from beer_caddy.joystick import UnixJoystick
from beer_caddy.tcp_client import TCPClient


class MySession(Session):
    """Contains all subsystems in this project. Allows for subsystems to reference each other via this object."""
    def __init__(self, args):
        """
        :param args: session arguments"
        """
        super().__init__()

        # properties for session arguments
        self.args = args

        # absolute paths for base and overlay config files
        self.base_config_path = os.path.abspath("config/base.yaml")
        self.overlay_config_path = os.path.abspath("config/remote.yaml")

        self.config = Config()  # contains the combined base and overlay configs
        self.base_config = Config()  # all parameters for project (the base config)
        self.overlay_config = Config()  # all projetc specific parameters
        self._load_config()  # pulls parameters from disk into config objects
        self.logger = self._init_log()  # initializes log object. Only call this once!!

        joystick_address = self.config.joystick.address if len(args.joystick_address) == 0 else args.joystick_address
        self.joystick = UnixJoystick(
            self.logger, joystick_address,
            self.config.joystick.button_mapping,
            self.config.joystick.axis_mapping,
        )

        self.tcp_client = TCPClient(
            self.config.tcp.host,
            self.config.tcp.port,
        )

        self.logger.info("Session initialized!")

    def start(self):
        """start relevant subsystems to fully initialize them"""
        self.joystick.start()
        self.tcp_client.start()
        self.logger.info("Session started!")

    def _load_config(self):
        """Load config files from disk using previously defined paths into session properties"""
        self.base_config = Config.from_file(self.base_config_path)
        self.overlay_config = Config.from_file(self.overlay_config_path)
        self.config.merge(self.base_config)
        self.config.merge(self.overlay_config)
    
    def _init_log(self):
        """Call the LoggerManager get_logger method to initialize logger. Only call once!"""
        return LoggerManager.get_logger(self.config.log)

    def stop(self, exception):
        """
        Fully shutdown appropriate subsystems. Signals slack bot that an exception has occurred.
        All of these happen to an extend if the python interpreter garbage collects them (calls __del__).
        So this method isn't 100% necessary.
        """
        self.joystick.stop()
        if exception is not None:
            self.logger.error(exception, exc_info=True)
        if isinstance(exception, ShutdownException):
            self.logger.warning("Shutdown function called. Shutting down everything.")
            subprocess.call("sudo shutdown -h now", shell=True)


async def update_joystick(session: MySession):
    """
    Task to call tunnel.update (arduino communications) in a loop
    :param session: instance of MySession
    """
    joystick = session.joystick
    tcp_client = session.tcp_client
    logger = session.logger
    while True:
        await joystick.update()
        await asyncio.sleep(0.05)
        vx = -joystick.deadband_axis("left/y", 0.05, 1.0)
        vy = -joystick.deadband_axis("left/x", 0.05, 1.0)
        vt = -joystick.deadband_axis("right/x", 0.05, 6.0)
        tcp_client.set_command(vx, vy, vt)
        tcp_client.update()
        if any(joystick.check_list(joystick.did_axis_change, "left/y", "right/x")):
            logger.info(f"{tcp_client.drive_command}")
        if all(joystick.check_list(joystick.is_button_down, "triggers/L1", "menu/Start")):
            tcp_client.set_enable(True)
            logger.info(f"Enabling")
        elif any(joystick.check_list(joystick.did_button_down, "triggers/L1", "triggers/R1")):
            tcp_client.set_enable(False)
            logger.info(f"Disabling")


def main():
    """Where the show starts and stops"""
    parser = argparse.ArgumentParser(description="home-delivery-bot")

    parser.add_argument("--device",
                        default="",
                        help="Set joystick address")
    cmd_args = parser.parse_args()

    args = RecursiveNamespace()
    args.joystick_address = cmd_args.device

    session = MySession(args)

    # add relevant asyncio tasks to run
    session.add_task(update_joystick(session))

    session.run()  # blocks until all tasks finish or an exception is raised


if __name__ == "__main__":
    main()
