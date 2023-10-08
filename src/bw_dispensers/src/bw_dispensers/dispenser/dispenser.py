from typing import Optional


class DispenseClientBase:
    def __init__(self) -> None:
        pass

    def start_dispense(self, dispenser_name) -> None:
        pass

    def is_done_dispensing(self) -> Optional[bool]:
        return False

    def close(self) -> None:
        pass
