import os
from pathlib import Path
import shutil
from bw_tools.training.pascal_voc import PascalVOCFrame
from bw_tools.training.yolo import YoloFrame
from .pascal_voc_collector import PascalVOCCollector
from .dataset_builder import DatasetBuilder, BACKGROUND_LABEL
from ..get_image_size import get_image_size

ANNOTATIONS = "labels"
JPEGIMAGES = "images"


class YoloDatasetBuilder(DatasetBuilder):
    def __init__(self, output_dir: Path, image_collector: PascalVOCCollector, labels: list, 
                 test_ratio=0.05, train_ratio=0.80, validation_ratio=0.15,
                 test_name="test", train_name="train", validation_name="val", dry_run=True):
        super(YoloDatasetBuilder, self).__init__(
            output_dir,
            test_ratio, train_ratio, validation_ratio,
            test_name, train_name, validation_name,
            dry_run
        )
        self.image_collector = image_collector
        self.frames = []
        self.frame_filenames = {}
        self.labels = labels

    def reset(self):
        # deletes all image and xml files under annotations and jpeg images respectively
        annotations_dir = self.output_dir / ANNOTATIONS
        images_dir = self.output_dir / JPEGIMAGES
        self.reset_dir(annotations_dir, ".txt")
        self.reset_dir(images_dir, ".jpg", ".jpeg")

    def build(self):
        self.frames = []
        
        label_to_identifier_map = {}
        frame_map = {}
        for frame in self.image_collector.iter():
            label = frame.objects[0].name
            if label not in label_to_identifier_map:
                label_to_identifier_map[label] = []
            label_to_identifier_map[label].append(frame.path)
            frame_map[frame.path] = frame
        image_sets = self.get_distributed_sets(label_to_identifier_map)

        test_count = len(image_sets[self.test_name])
        train_count = len(image_sets[self.train_name])
        validation_count = len(image_sets[self.validation_name])
        total_count = test_count + train_count + validation_count
        print(
            f"Total {total_count}:\n"
            f"\tTest: {test_count}\t{test_count / total_count:0.2f}\n"
            f"\tTrain: {train_count}\t{train_count / total_count:0.2f}\n"
            f"\tValidation: {validation_count}\t{validation_count / total_count:0.2f}"
        )

        for set_key in image_sets.keys():
            for label in self.labels:
                self.makedir(self.output_dir / ANNOTATIONS / set_key)
                self.makedir(self.output_dir / JPEGIMAGES / set_key)

        for key, image_set in image_sets.items():
            for frame_path in image_set:
                frame = frame_map[frame_path]
                self.frames.append(frame)
                self.copy_annotation(key, frame)

        self.write_labels()

    def write_labels(self):
        if BACKGROUND_LABEL in self.labels:
            self.labels.remove(BACKGROUND_LABEL)
        labels_path = self.output_dir / "classes.txt"
        print("Writing labels to %s" % labels_path)
        self.write_list(labels_path, self.labels)

    def copy_annotation(self, subdirectory: str, frame: PascalVOCFrame):
        if frame.filename not in self.frame_filenames:
            self.frame_filenames[frame.filename] = 0
        filename_count = self.frame_filenames[frame.filename]
        self.frame_filenames[frame.filename] += 1
        width, height = get_image_size(frame.path)
        frame.width = width
        frame.height = height

        annotation_dir = self.output_dir / ANNOTATIONS / subdirectory
        images_dir = self.output_dir / JPEGIMAGES / subdirectory
        self.makedir(annotation_dir)
        self.makedir(images_dir)
        new_image_path = images_dir / frame.filename
        if filename_count > 0:
            name = new_image_path.stem
            ext = new_image_path.suffix
            new_image_path = new_image_path.parent / ("%s-%05d%s" % (name, filename_count, ext))

        new_frame_path = annotation_dir / os.path.basename(frame.frame_path)
        if filename_count > 0:
            name = new_frame_path.stem
            ext = ".txt"
            new_frame_path = new_frame_path.parent / ("%s-%05d%s" % (name, filename_count, ext))
        else:
            name = new_frame_path.stem
            ext = ".txt"
            new_frame_path = new_frame_path.parent / ("%s%s" % (name, ext))

        print("Copying image %s -> %s%s" % (frame.path, new_image_path,
                                            (". Adding count: %05d" % filename_count) if filename_count > 0 else ""))
        if not self.dry_run:
            shutil.copy(frame.path, new_image_path)

        frame.set_path(new_image_path)
        print("Copying annotation %s -> %s" % (frame.frame_path, new_frame_path))
        frame.frame_path = str(new_frame_path.name)
        yolo_frame = YoloFrame.from_pascal_voc(frame, self.labels)
        if not self.dry_run:
            yolo_frame.write(str(new_frame_path))
