import os
import argparse
import requests
from PIL import Image

image_formats = {
    "tag16h5": "tag16_05_%05d.png",
    "tag25h9": "tag25_09_%05d.png",
    "tag36h11": "tag36_11_%05d.png",
    "tagCircle21h7": "tag21_07_%05d.png",
    "tagCircle49h12": "tag49_12_%05d.png",
    "tagCustom48h12": "tag48_12_%05d.png",
    "tagStandard41h12": "tag41_12_%05d.png",
    "tagStandard52h13": "tag52_13_%05d.png",
}

parser = argparse.ArgumentParser(description="tag")

parser.add_argument("tag",
                    choices=list(image_formats.keys()),
                    help="tag family")
parser.add_argument("code",
                    type=int,
                    help="tag embedded code")
parser.add_argument("-p", "--pixels_per_mm",
                    type=float,
                    default=10.0,
                    help="pixels per mm")
parser.add_argument("-l", "--length_mm",
                    type=float,
                    default=200.0,
                    help="length mm")

args = parser.parse_args()

tag_family = args.tag
code = args.code

px_per_mm = args.pixels_per_mm
length_mm = args.length_mm

file_name = image_formats[tag_family] % code

url = f"https://github.com/AprilRobotics/apriltag-imgs/raw/master/{tag_family}/{file_name}"

img_data = requests.get(url).content
with open(file_name, 'wb') as handler:
    handler.write(img_data)
print("Downloaded image to", file_name)

new_size_px = int(length_mm * px_per_mm)
print(f"Resizing to {new_size_px} px")

pdf_path = os.path.splitext(file_name)[0] + ".pdf"

dpi = px_per_mm * 25.4

image = Image.open(file_name)
image = image.convert('RGB')
image = image.resize((new_size_px, new_size_px), Image.NEAREST)
image.save(pdf_path, resolution=dpi)
print("PDF path:", pdf_path)