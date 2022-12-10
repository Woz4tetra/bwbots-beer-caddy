rosservice call /zed/start_yolo_object_detection "{confidence: ${1:-85.0}, max_range: ${2:-40.0}, tracking: true}"
