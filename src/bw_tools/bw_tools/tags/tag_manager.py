from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import yaml
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Quaternion

from bw_tools.dataclass_deserialize import dataclass_deserialize
from bw_tools.typing.math import from_quat


@dataclass(frozen=True, eq=True)
class AprilTagBundleComponentConfig:
    id: int
    size: float
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float


@dataclass(frozen=True, eq=True)
class AprilTagBundleConfig:
    name: str
    layout: List[AprilTagBundleComponentConfig]


@dataclass(frozen=True, eq=True)
class AprilTagSingleConfig:
    id: int
    size: float
    name: str


@dataclass(frozen=True, eq=True)
class AprilTagSingle:
    metadata: AprilTagSingleConfig
    detection: AprilTagDetection


@dataclass(frozen=True, eq=True)
class AprilTagBundle:
    metadata: AprilTagBundleConfig
    detection: AprilTagDetection


@dataclass(frozen=True, eq=True)
class AprilTagNodeConfig:
    tag_family: str
    tag_threads: int
    tag_decimate: float
    tag_blur: float
    tag_refine_edges: int
    tag_debug: int
    max_hamming_dist: int
    remove_duplicates: bool
    publish_tf: bool
    transport_hint: str
    standalone_tags: List[AprilTagSingleConfig]
    tag_bundles: List[AprilTagBundleConfig]


class TagManager:
    def __init__(self, apriltag_config_path: str) -> None:
        self.config = self.load_config(apriltag_config_path)
        self.single_ids: Dict[int, AprilTagSingleConfig] = {}
        for tag in self.config.standalone_tags:
            if tag.name in self.single_ids:
                raise ValueError(f"Duplicate tag IDs {tag.id} ({tag.name})")
            self.single_ids[tag.id] = tag
        tag_names = [tag.name for tag in self.config.standalone_tags]
        assert len(tag_names) == len(set(tag_names)), "Duplicate tag names detected"

        self.bundle_ids: Dict[Tuple[int, ...], AprilTagBundleConfig] = {}
        for bundle in self.config.tag_bundles:
            ids = tuple(sorted([component.id for component in bundle.layout]))
            if ids in self.bundle_ids:
                raise ValueError(f"Duplicate bundle IDs {ids} ({bundle.name})")
            self.bundle_ids[ids] = bundle
        bundle_names = [bundle.name for bundle in self.config.tag_bundles]
        assert len(bundle_names) == len(set(bundle_names)), "Duplicate bundle names detected"

    def is_name_tag(self, name: str) -> bool:
        return any([name in tag.name for tag in self.config.standalone_tags])

    def is_name_bundle(self, name: str) -> bool:
        return any([name in bundle.name for bundle in self.config.tag_bundles])

    def load_config(self, path: str) -> AprilTagNodeConfig:
        with open(path) as file:
            config = yaml.safe_load(file)
        return dataclass_deserialize(AprilTagNodeConfig, config)

    def get_matching_tags(self, msg: AprilTagDetectionArray) -> List[AprilTagSingle]:
        assert msg.detections is not None
        result = []
        for detection in msg.detections:
            if len(detection.id) != 1:
                continue
            if detection.id[0] in self.single_ids:
                result.append(AprilTagSingle(self.single_ids[detection.id[0]], detection))
        return result

    def get_tag_by_name(self, name: str, msg: AprilTagDetectionArray) -> Optional[AprilTagSingle]:
        matching = self.get_matching_tags(msg)
        result = [tag for tag in matching if tag.metadata.name == name]
        if len(result) == 0:
            return None
        else:
            assert len(result) == 1
            return result[0]

    def get_matching_bundles(self, msg: AprilTagDetectionArray) -> List[AprilTagBundle]:
        assert msg.detections is not None
        result = []
        for detection in msg.detections:
            if len(detection.id) <= 1:
                continue
            ids = tuple(sorted(detection.id))
            if ids in self.bundle_ids:
                result.append(AprilTagBundle(self.bundle_ids[ids], detection))
        return result

    def get_bundle_by_name(self, name: str, msg: AprilTagDetectionArray) -> Optional[AprilTagBundle]:
        matching = self.get_matching_bundles(msg)
        result = [bundle for bundle in matching if bundle.metadata.name == name]
        if len(result) == 0:
            return None
        else:
            assert len(result) == 1
            return result[0]

    @classmethod
    def rotate_tag(
        cls,
        tag_orientation: Quaternion,
        rotate_quat: Tuple[float, float, float, float] = (0.5, -0.5, -0.5, -0.5),
    ) -> Quaternion:
        rotate_mat = from_quat(rotate_quat)
        tag_mat = from_quat(
            (
                tag_orientation.x,
                tag_orientation.y,
                tag_orientation.z,
                tag_orientation.w,
            )
        )
        rotated_tag = tag_mat * rotate_mat
        return Quaternion(*rotated_tag.as_quat())
