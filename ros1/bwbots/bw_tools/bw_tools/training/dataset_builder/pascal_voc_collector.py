import os
import cv2
from bw_tools.training.get_image_size import get_image_metadata
from bw_tools.training.pascal_voc import PascalVOCFrame
from bw_tools.training.dataset_builder.collector import Collector


class PascalVOCCollector(Collector):
    def __init__(self, base_dir):
        super().__init__()
        self.base_dir = base_dir
        self.load_annotations()

    def load_annotations(self):
        for dirpath, dirnames, filenames in os.walk(self.base_dir):
            for filename in filenames:
                if filename.endswith(".xml"):
                    path = os.path.join(dirpath, filename)
                    frame = PascalVOCFrame.from_path(path)
                    if os.path.isfile(frame.path):
                        image_path = frame.path
                    else:
                        image_path = os.path.join(dirpath, frame.filename)
                        frame.set_path(image_path)
                    if not os.path.isfile(image_path):
                        raise FileNotFoundError("Image file corresponding to '%s' annotation could not be found" % path)
                    image_metadata = get_image_metadata(image_path)

                    self.frames.append(frame)
                    self.images.append(None)
                    self.metadata.append(image_metadata)
