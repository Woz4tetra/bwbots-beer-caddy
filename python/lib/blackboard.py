
from typing import Any, Callable, Optional


class Blackboard:
    def __init__(self) -> None:
        self._data = {}
        self._listeners = {}

    def add_listener(self, key: str, callback: Callable):
        if key not in self._listeners:
            self._listeners[key] = []
        self._listeners[key].append(callback)

    def publish(self, key: str, value: Any):
        self._data[key] = value
        if key in self._listeners:
            for callback in self._listeners[key]:
                callback(value)

    def get(self, key: str, default=None) -> Optional[Any]:
        return self._data.get(key, default)
