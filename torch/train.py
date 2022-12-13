import os
from yolov5 import train

train.run(
    data=os.path.abspath("outputs/cargo_2022.yaml"),
    imgsz=640,
    epochs=300,
    weights=os.path.abspath("resources/pretrained/yolov5n.pt"),
    device=0,
    batch_size=4,
    project=os.path.abspath("outputs/cargo_2022_train"),
    multi_scale=True
)
