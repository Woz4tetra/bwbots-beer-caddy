import os
from typing import Dict, List
import cv2
from bw_tools.training.get_image_size import get_image_metadata
from bw_tools.training.pascal_voc import PascalVOCFrame, PascalVOCObject


class Collector:
    def __init__(self):
        self.frames = []
        self.images = []
        self.metadata = []

    def add_detection(self, image_path: str, objects: Dict[str, List]):
        image_metadata = get_image_metadata(image_path)
        path, image_type, file_size, width, height = image_metadata

        frame = PascalVOCFrame()
        frame.frame_path = os.path.splitext(image_path)[0] + ".xml"
        frame.set_path(image_path)
        frame.width = width
        frame.height = height
        frame.depth = 3
        for label, bounding_boxes in objects.items():
            for box in bounding_boxes:
                obj = PascalVOCObject()
                obj.bndbox = box
                obj.name = label
                if obj.is_out_of_bounds(width, height):
                    continue
                if obj.is_truncated(width, height):
                    obj.bndbox = box
                    
                frame.add_object(obj)

        if len(frame.objects) > 0:
            self.frames.append(frame)
            self.images.append(None)
            self.metadata.append(image_metadata)

    def _load_image(self, image_path, index):
        image = cv2.imread(image_path)
        self.images[index] = image

    def iter(self, include_image=False, include_metadata=False):
        assert len(self.frames) == len(self.images) == len(self.metadata)
        for index in range(len(self.frames)):
            frame = self.frames[index]
            result = [frame]
            if include_image:
                image = self.images[index]
                if image is None:
                    self._load_image(frame.path, index)
                    image = self.images[index]
                result.append(image)
            if include_metadata:
                image_metadata = self.metadata[index]
                result.append(image_metadata)
            if len(result) == 1:
                yield result[0]
            else:
                yield tuple(result)
