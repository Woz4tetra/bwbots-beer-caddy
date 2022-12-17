import os
import sys
import cv2
import tqdm
import math
import pickle
import random
import requests
import matplotlib
import numpy as np
import shapely.geometry
from shapely.geometry import Polygon
from matplotlib import pyplot as plt
import matplotlib.patches
from matplotlib.collections import PatchCollection
from pathlib import Path
from pycocotools.coco import COCO

sys.path.insert(0, "../ros1/bwbots/bw_tools")

from bw_tools.training.dataset_builder.collector import Collector
from bw_tools.training.dataset_builder.yolo_dataset_builder import YoloDatasetBuilder

matplotlib.use("TkAgg")


def download_image(url, path):
    # print(f"Downloading {url}")
    img_data = requests.get(url).content
    with open(path, 'wb') as handler:
        handler.write(img_data)


def download_coco_dataset():
    annotation_file = "dataset/coco2017/annotations/person_keypoints_train2017.json"
    images_dir = "dataset/coco2017/images"

    coco = COCO(annotation_file)
    category_ids = coco.getCatIds(catNms=['person'])
    image_ids = coco.getImgIds(catIds=category_ids)
    images_info = coco.loadImgs(image_ids)

    if not os.path.isdir(images_dir):
        os.makedirs(images_dir)
    with tqdm.tqdm(total=len(images_info)) as pbar:
        for image_info in images_info:
            image_path = os.path.join(images_dir, image_info["file_name"])
            if os.path.isfile(image_path):
                continue
            download_image(image_info["coco_url"], image_path)
            pbar.update(1)


def show_keypoints():
    annotation_file = "dataset/coco2017/annotations/person_keypoints_train2017.json"
    images_dir = "dataset/coco2017/images"

    coco = COCO(annotation_file)
    category_ids = coco.getCatIds(catNms=['person'])
    image_ids = coco.getImgIds(catIds=category_ids)
    images_info = coco.loadImgs(image_ids)

    # image_info = images_info[3]
    image_info = {
        'id': 178647,
        'file_name': "000000178647.jpg"
    }
    annotation_ids = coco.getAnnIds(imgIds=image_info['id'], catIds=category_ids, iscrowd=None)
    annotations = coco.loadAnns(annotation_ids)
    categories = coco.loadCats(category_ids)
    

    image_path = os.path.join(images_dir, image_info["file_name"])
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    plt.imshow(image)
    coco.showAnns(annotations)
    ax = plt.gca()
    targets = ["right_wrist", "left_wrist", "right_elbow", "left_elbow", "right_ankle", "left_ankle", "right_knee", "left_knee"]

    named_keypoints = categories[0]['keypoints']
    for annotation in annotations:
        keypoints = annotation['keypoints']
        for index in range(0, len(keypoints), 3):
            keypoint_index = index // 3
            x = keypoints[index]
            y = keypoints[index + 1]
            visible = keypoints[index + 2]
            if visible != 2:
                continue
            point_name = named_keypoints[keypoint_index]

            ax.annotate(
                point_name, xy=(x, y),
                arrowprops=dict(facecolor='black', shrink=0.05),
                horizontalalignment='right', verticalalignment='top',
            )
            if point_name not in targets:
                continue
            
            c = (np.random.random((1, 3))*0.6+0.4).tolist()[0]
            segmentation = annotation['segmentation'][0]
            polygon_array = np.array(segmentation).reshape((int(len(segmentation)/2), 2))

            bb = best_fit_bounding_box(x, y, polygon_array)
            if bb is not None:
                bb_x0, bb_y0, bb_x1, bb_y1 = bb
                plt.plot([bb_x0, bb_x0, bb_x1, bb_x1, bb_x0], [bb_y0, bb_y1, bb_y1, bb_y0, bb_y0], '-', color=c)
        
    archive = {
        "named_keypoints": named_keypoints,
        "annotations": annotations,
        "image": image,
    }
    with open("dataset/archive.pkl", 'wb') as file:
        pickle.dump(archive, file)

    plt.show()


def compute_area_captured(vars, *args):
    width, height = vars
    x, y, seg_polygon, max_width, max_height = args
    box_polygon = Polygon([
        (x - width / 2, y - height / 2),
        (x + width / 2, y - height / 2),
        (x + width / 2, y + height / 2),
        (x - width / 2, y + height / 2),
    ])
    value = box_polygon.intersection(seg_polygon).area / box_polygon.area
    return value

def best_fit_bounding_box(x, y, segmentation):
    seg_polygon = Polygon(segmentation)
    point = shapely.geometry.Point(x, y)
    pol_ext = shapely.geometry.LinearRing(seg_polygon.exterior.coords)
    pt1 = pol_ext.interpolate(pol_ext.project(point))
    
    nearest_x = pt1.coords.xy[0][0]
    nearest_y = pt1.coords.xy[1][0]
    
    dx = nearest_x - x
    dy = nearest_y - y
    distance = math.sqrt(dx * dx + dy * dy)
    
    circle_limit = point.buffer(distance, resolution=2)
    
    # intersected = circle_limit.intersection(seg_polygon)
    if circle_limit.area < 10.0:
        return None
    else:
        return circle_limit.bounds
    

def crop_annotation(annotations, crop_percents, image=None):
    top_percent, bottom_percent, left_percent, right_percent = crop_percents
    height, width = image.shape[0:2]
    assert top_percent < bottom_percent
    assert left_percent < right_percent

    top_index = 0 if top_percent == 0.0 else int(height * top_percent)
    bottom_index = height if bottom_percent == 1.0 else int(height * bottom_percent)
    left_index = 0 if left_percent == 0.0 else int(width * left_percent)
    right_index = width if right_percent == 1.0 else int(width * right_percent)

    if image is not None:
        crop_image = image[top_index:bottom_index, left_index:right_index]
    else:
        crop_image = None
    
    right_index -= left_index
    bottom_index -= top_index
    
    bounding_boxes = []
    for annotation in annotations:
        if type(annotation['segmentation']) != list:
            continue
        bb_x0, bb_y0, bb_w, bb_h = annotation['bbox']

        bb_x0 -= left_index
        bb_y0 -= top_index

        bb_x1 = bb_x0 + bb_w
        bb_y1 = bb_y0 + bb_h

        if bb_x0 < 0:
            bb_x0 = 0
        if bb_y0 < 0:
            bb_y0 = 0

        if bb_x1 < 0 or bb_x0 >= right_index:
            continue
        if bb_y1 < 0 or bb_y0 >= bottom_index:
            continue

        if bb_x1 >= right_index:
            bb_x1 = float(right_index - 1)
        if bb_y1 >= bottom_index:
            bb_y1 = float(bottom_index - 1)

        bb = bb_x0, bb_y0, bb_x1, bb_y1
        if bb_x0 >= bb_x1 or bb_y0 >= bb_y1:
            continue
        if bb_x1 - bb_x0 < 10.0 or bb_y1 - bb_y0 < 10.0:
            continue
        bounding_boxes.append(bb)
    return crop_image, bounding_boxes


def test_boundingbox_keypoints():
    with open("dataset/archive.pkl", 'rb') as file:
        archive = pickle.load(file)
    annotations = archive["annotations"]
    image = archive["image"]
    plt.figure(1)
    ax = plt.gca()
    color = []
    crop = (0.15563782030053902, 0.7248561404569517, 0.0, 1.0)
    # crop = (0.0, 1.0, 0.0, 1.0)
    image, bounding_boxes = crop_annotation(annotations, crop, image)
    ax.imshow(image)
    for box in bounding_boxes:
        bb_x0, bb_y0, bb_x1, bb_y1 = box
        c = (np.random.random((1, 3))*0.6+0.4).tolist()[0]
        plt.plot([bb_x0, bb_x0, bb_x1, bb_x1, bb_x0], [bb_y0, bb_y1, bb_y1, bb_y0, bb_y0], '-', color=c)
        color.append(c)

    plt.show()


def format_detect_dataset():
    collector = Collector()
    annotation_file = "dataset/coco2017/annotations/person_keypoints_train2017.json"
    images_dir = "dataset/coco2017/images"

    coco = COCO(annotation_file)
    category_ids = coco.getCatIds(catNms=['person'])
    image_ids = coco.getImgIds(catIds=category_ids)
    images_info = coco.loadImgs(image_ids)
    
    def add_detections(image_path, label, bounding_boxes):
        objects = {}
        for bounding_box in bounding_boxes:
            if label not in objects:
                objects[label] = []
            objects[label].append(bounding_box)
        collector.add_detection(image_path, objects)
    
    def randomize_crop(crop, noise):
        new_crop = []
        for dim in crop:
            if not (dim == 1.0 or dim == 0.0):
                dim += 2.0 * (np.random.random() - 0.5) * noise
            new_crop.append(dim)
        return tuple(new_crop)
    
    crops = (
        (0.3, 1.0, 0.0, 1.0),
        (0.5, 1.0, 0.0, 1.0),
        (0.25, 0.75, 0.0, 1.0),
        (0.4, 0.9, 0.0, 1.0),
    )
    crop_noise = 0.1
    num_samples = 10000
    
    random.seed(4500)
    all_indices = list(range(len(images_info)))
    random.shuffle(all_indices)
    selected_indices = all_indices[0:num_samples]
    
    label = 'person'
    labels = [label]
    with tqdm.tqdm(total=len(images_info)) as pbar:
        for index in selected_indices:
            image_info = images_info[index]
            annotation_ids = coco.getAnnIds(imgIds=image_info['id'], catIds=category_ids, iscrowd=None)
            annotations = coco.loadAnns(annotation_ids)

            image_path = os.path.join(images_dir, image_info["file_name"])
            if not os.path.isfile(image_path):
                continue

            image = cv2.imread(image_path)
            _, bounding_boxes = crop_annotation(annotations, (0.0, 1.0, 0.0, 1.0), image)
            add_detections(image_path, label, bounding_boxes)

            for index, crop in enumerate(crops):
                crop = randomize_crop(crop, crop_noise)
                cropped_image, bounding_boxes = crop_annotation(annotations, crop, image)
                if len(bounding_boxes) == 0:
                    continue
                name, ext = os.path.splitext(image_path)
                new_path = f'{name}-{index}{ext}'
                cv2.imwrite(new_path, cropped_image)
                add_detections(new_path, label, bounding_boxes)
            
            pbar.update(1)

    dataset = YoloDatasetBuilder(Path("outputs/person_dataset"), collector, list(labels), dry_run=False)
    dataset.reset()
    dataset.build()


if __name__ == '__main__':
    format_detect_dataset()
    # download_coco_dataset()
    # show_keypoints()
    # test_boundingbox_keypoints()
