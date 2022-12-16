# https://docs.ultralytics.com/tutorials/torchscript-onnx-coreml-export/
import os
import time

from yolov5 import export

base_dir = "outputs/person_train"

last_modified_time = time.time()
last_modified_path = ""

for dirname in os.listdir(base_dir):
    path = os.path.join(base_dir, dirname)
    modified_time = os.path.getmtime(path)
    if modified_time < last_modified_time:
        last_modified_time = modified_time
        last_modified_path = path

export.run(
    data=os.path.abspath("person.yaml"),
    weights=os.path.abspath(os.path.join(last_modified_path, "weights", "best.pt")),
    device=0,
    imgsz=(640, 640),
)
