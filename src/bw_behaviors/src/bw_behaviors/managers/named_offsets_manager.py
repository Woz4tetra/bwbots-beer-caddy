from typing import Dict, Optional, Tuple

from bw_tools.robot_state import Pose2d


class NamedOffsetsManager:
    def __init__(self, offsets: Optional[Dict[str, Tuple[float, ...]]]) -> None:
        if offsets is None:
            offsets = {}
        self.named_offsets: Dict[str, Pose2d] = {
            name: Pose2d(
                x=offset[0],
                y=offset[1],
                theta=offset[2],
            )
            for name, offset in offsets.items()
        }

    def get(self, name: str) -> Pose2d:
        return self.named_offsets[name]
