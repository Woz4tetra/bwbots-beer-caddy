# https://docs.ultralytics.com/tutorials/torchscript-onnx-coreml-export/
import os
import sys
import yaml

from util import get_best_model

yolov5_path = sys.argv[1]

sys.path.insert(0, yolov5_path)

import yolov5
import yolov5.export

model_name = "person_2022-12-18.torchscript"
classes_name = "person.names"

base_dir = "outputs/person_train"

last_modified_path = get_best_model(base_dir)

weights_dir = os.path.abspath(os.path.join(last_modified_path, "weights"))
data_path = os.path.abspath("person.yaml")

yolov5.export.run(
    data=data_path,
    weights=os.path.join(weights_dir, "best.pt"),
    device=0,
    imgsz=(640, 640),
)

old_torchscript = os.path.join(weights_dir, "best.torchscript")
new_torchscript = os.path.join(weights_dir, model_name)
os.rename(old_torchscript, new_torchscript)

names_path = os.path.join(weights_dir, classes_name)

with open(data_path) as file:
    config = yaml.load(file)
    classes = config["names"]

with open(names_path, 'w') as file:
    file.write("\n".join(classes))


print(f"Model exported to {new_torchscript}")
print(f"Names written to {names_path}")
