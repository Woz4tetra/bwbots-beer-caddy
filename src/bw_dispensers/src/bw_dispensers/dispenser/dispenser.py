from typing import Optional


class DispenseClientBase:
    def __init__(self) -> None:
        pass

    def has_drink(self) -> Optional[bool]:
        return None

    def start_dispense(self, dispenser_name) -> bool:
        return False

    def is_done_dispensing(self) -> Optional[bool]:
        return False

    def close(self) -> None:
        pass