import os
import sys
import cv2

from util import get_best_model

yolov5_path = sys.argv[1]

sys.path.insert(0, os.path.join(yolov5_path, "yolov5"))
sys.path.insert(0, yolov5_path)

from bw_tools.yolo.detector import YoloDetector

model_device = "0"
base_dir = "outputs/person_train"
images_dir = "outputs/person_dataset/images/test"

model_path = os.path.join(get_best_model(base_dir), "weights", "best.pt")

image_width = 672
image_height = 376
confidence_threshold = 0.4
nms_iou_threshold = 0.45
max_detections = 100
report_loop_times = True
publish_overlay = True

yolo = YoloDetector(
    model_device, model_path, image_width, image_height,
    confidence_threshold, nms_iou_threshold, max_detections,
    report_loop_times, publish_overlay
)

window_name = "detections"

start_index = 44

count = 0
for dirpath, dirnames, filenames in os.walk(images_dir):
    for filename in filenames:
        if not filename.endswith(".jpg"):
            continue
        path = os.path.join(dirpath, filename)
        print(f"Image: #{count}. {path}")
        count += 1
        if count - 1 < start_index:
            continue
        image = cv2.imread(path)
        detections, overlay_image = yolo.detect(image)
        cv2.imshow(window_name, overlay_image)
        key = chr(cv2.waitKey(-1) & 0xff)
        if key == 'q':
            sys.exit(0)
