import logging
import re
import subprocess
import time
from enum import Enum, auto

from config import Config, load_config
from systemd.journal import JournalHandler

logger = logging.getLogger('hotspot')
logger.addHandler(JournalHandler())
logger.setLevel(logging.DEBUG)


class ConnectionStates(Enum):
    NOT_CONNECTED = auto()
    HOTSPOT_ACTIVE = auto()
    WIFI_ACTIVE = auto()


class HotspotController:
    def __init__(self, config: Config) -> None:
        self.config = config
        self.up_pattern = r"(.+): .+ state (UP|DOWN) mode.+"

    def get_state(self) -> ConnectionStates:
        regex = r"^([\S\s]+)\s{2}(\S+)\s{2}(\S+)\s+(\S+)\s*"
        proc = subprocess.Popen(["nmcli", "con"], stdout=subprocess.PIPE)
        output, error = proc.communicate()
        state = ConnectionStates.NOT_CONNECTED
        for line in output.decode().splitlines()[1:]:
            match = re.search(regex, line)
            if not match:
                continue
            name = match.group(1).strip()
            # group 2 is UUID
            # group 3 is connection type
            device = match.group(4).strip()
            if device == self.config.wifi_interface:
                if name == self.config.hotspot_name:
                    return ConnectionStates.HOTSPOT_ACTIVE
                else:
                    state = ConnectionStates.WIFI_ACTIVE
        return state

    def set_hotspot(self, state: bool) -> None:
        if state:
            command = "up"
        else:
            command = "down"
        subprocess.run(["nmcli", "con", command, self.config.hotspot_name])


def restart_main_service(config: Config) -> None:
    subprocess.run(["systemctl", "restart", config.main_service])


def main():
    logger.info("Hotspot controller starting")
    config = load_config()
    manager = HotspotController(config)

    start_time = time.monotonic()
    manager.set_hotspot(False)
    while time.monotonic() - start_time < config.waiting_period:
        time.sleep(0.5)
        logger.info("State is %s" % manager.get_state())
        if manager.get_state() == ConnectionStates.WIFI_ACTIVE:
            logger.info("Wifi is connected, exiting")
            return
    if manager.get_state() == ConnectionStates.NOT_CONNECTED:
        manager.set_hotspot(True)
    time.sleep(1.0)
    if manager.get_state() == ConnectionStates.HOTSPOT_ACTIVE:
        logger.info("Hotspot started successfully!")
    else:
        logger.info("Failed to start hotspot!")


if __name__ == "__main__":
    main()
