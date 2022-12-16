import os
import sys
import yaml
from tempfile import NamedTemporaryFile

yolov5_path = sys.argv[1]

sys.path.insert(0, yolov5_path)

import yolov5
import yolov5.train

data_path = os.path.abspath("person.yaml")

with NamedTemporaryFile('w') as tmp:
    with open(data_path, 'r') as file:
        config = yaml.load(file)
    base_dir = config["path"]
    config["path"] = os.path.abspath(os.path.join(base_dir, config.get("path", '')))
    config["train"] = os.path.abspath(os.path.join(base_dir, config.get("train", '')))
    config["val"] = os.path.abspath(os.path.join(base_dir, config.get("val", '')))
    config["test"] = os.path.abspath(os.path.join(base_dir, config.get("test", '')))
    yaml.dump(config, tmp)

    model_path = os.path.join(os.path.dirname(yolov5.__file__), "models", "yolov5n.yaml")
    yolov5.train.run(
        data=tmp.name,
        imgsz=640,
        epochs=300,
        cfg=model_path,
        weights='',
        device=0,
        batch_size=4,
        project=os.path.abspath("outputs/person_train"),
        multi_scale=True
    )
