import os
import sys
import cv2
import tqdm
import math
import pickle
import requests
import matplotlib
import numpy as np
import scipy.optimize
from pprint import pprint
import shapely.geometry
from shapely.geometry import Polygon
from shapely.ops import nearest_points
from matplotlib import pyplot as plt
import matplotlib.patches
from matplotlib.collections import PatchCollection
from pathlib import Path
from pycocotools.coco import COCO

sys.path.insert(0, "../ros1/bwbots/bw_tools")

from tj2_tools.training.detect_collector import DetectCollector
from tj2_tools.training.dataset_builder.yolo_dataset_builder import YoloDatasetBuilder

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
    
    image_info = images_info[3]
    annotation_ids = coco.getAnnIds(imgIds=image_info['id'], catIds=category_ids, iscrowd=None)
    annotations = coco.loadAnns(annotation_ids)
    categories = coco.loadCats(category_ids)
    

    image_path = os.path.join(images_dir, image_info["file_name"])
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    plt.imshow(image)
    coco.showAnns(annotations)
    ax = plt.gca()

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
        
    archive = {
        "named_keypoints": named_keypoints,
        "annotations": annotations,
        "image": image,
    }
    with open("dataset/archive.pkl", 'wb') as file:
        pickle.dump(archive, file)

    plt.show()


def format_detect_dataset():
    base_dir = "resources/labeled-images"

    collector = DetectCollector(base_dir)
    dataset = YoloDatasetBuilder(Path("outputs/person_dataset"), collector, ["person"],
                                 dry_run=False)
    dataset.reset()
    dataset.build()


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

def best_fit_bounding_box(x, y, width, height, segmentation):
    seg_polygon = Polygon(segmentation)
    point = shapely.geometry.Point(x, y)
    # pt1, pt2 = nearest_points(seg_polygon, point)
    pol_ext = shapely.geometry.LinearRing(seg_polygon.exterior.coords)
    pt1 = pol_ext.interpolate(pol_ext.project(point))
    
    nearest_x = pt1.coords.xy[0][0]
    nearest_y = pt1.coords.xy[1][0]
    
    dx = nearest_x - x
    dy = nearest_y - y
    distance = math.sqrt(dx * dx + dy * dy)
    distance *= 2.5
    
    circle_limit = point.buffer(distance, resolution=2)
    
    chopped_array = []
    xs = seg_polygon.exterior.coords.xy[0].tolist()
    ys = seg_polygon.exterior.coords.xy[1].tolist()
    intersected = circle_limit.intersection(seg_polygon)
    if type(intersected) == shapely.geometry.MultiPolygon:
        return None
    # for seg_pt in zip(xs, ys):
    #     if shapely.geometry.Point(*seg_pt).within(circle_limit):
    #         chopped_array.append(seg_pt)
    xs = intersected.exterior.coords.xy[0].tolist()
    ys = intersected.exterior.coords.xy[1].tolist()
    for seg_pt in zip(xs, ys):
        chopped_array.append(seg_pt)
    if len(chopped_array) == 0:
        raise RuntimeError("All segmentation points filtered out!")
    contour = np.array([chopped_array], dtype=np.int32)
    ellipse = cv2.fitEllipse(contour)
    new_width = ellipse[1][0]
    new_height = ellipse[1][1]
    return (
        x - new_width / 2,
        y - new_height / 2,
        x + new_width / 2,
        y + new_height / 2,
    ), ellipse, contour
    


def test_boundingbox_keypoints():
    with open("dataset/archive.pkl", 'rb') as file:
        archive = pickle.load(file)
    named_keypoints = archive["named_keypoints"]
    annotations = archive["annotations"]
    image = archive["image"]
    plt.figure(1)
    ax = plt.gca()
    polygons = []
    color = []
    annotation = annotations[2]
    # for annotation in annotations:
    #     if type(annotation['segmentation']) == list:
    #         break
    if type(annotation['segmentation']) != list:
        raise RuntimeError("Failed to find segmentation")
    keypoints = annotation['keypoints']
    segmentation = annotation['segmentation'][0]
    polygon_array = np.array(segmentation).reshape((int(len(segmentation)/2), 2))
    
    targets = ["right_wrist", "left_wrist", "right_elbow", "left_elbow", "right_ankle", "left_ankle", "right_knee", "left_knee"]
    
    points = []
    c = (np.random.random((1, 3))*0.6+0.4).tolist()[0]
    for index in range(0, len(keypoints), 3):
        keypoint_index = index // 3
        x = keypoints[index]
        y = keypoints[index + 1]
        visible = keypoints[index + 2]
        if visible != 2:
            continue
        point_name = named_keypoints[keypoint_index]
        if point_name not in targets:
            continue

        ax.annotate(
            point_name, xy=(x, y),
            arrowprops=dict(facecolor='black', shrink=0.05),
            horizontalalignment='right', verticalalignment='top',
        )
        points.append((x, y))
        
        result = best_fit_bounding_box(x, y, image.shape[1], image.shape[0], polygon_array)
        if result is None:
            continue
        bb, ellipse, contour = result
        image = cv2.ellipse(image, ellipse, (0, 0, 255))
        cv2.drawContours(image, contour, 0, (0, 255, 0), 1)
        bb_x0, bb_y0, bb_x1, bb_y1 = bb
        plt.plot([bb_x0, bb_x0, bb_x1, bb_x1, bb_x0], [bb_y0, bb_y1, bb_y1, bb_y0, bb_y0], '-', color=c)
        # break
    ax.imshow(image)
    points = np.array(points)
    
    polygons.append(matplotlib.patches.Polygon(polygon_array))
    color.append(c)
    
    p = PatchCollection(polygons, facecolor=color, linewidths=0, alpha=0.4)
    ax.add_collection(p)
    p = PatchCollection(polygons, facecolor='none', edgecolors=color, linewidths=2)
    ax.add_collection(p)
    plt.plot(points[:, 0], points[:, 1], '.', markersize=10, color=c)

    plt.show()

if __name__ == '__main__':
    # format_detect_dataset()
    # download_coco_dataset()
    # show_keypoints()
    test_boundingbox_keypoints()
