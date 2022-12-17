
class YoloObject:
    def __init__(self):
        self.bndbox = [0.0, 0.0, 0.0, 0.0]  # [center x, center y, width, height] (normalized to image size)
        self.class_index = 0

    @classmethod
    def from_pascal_voc(cls, obj, class_mapping: list, image_width, image_height):
        self = cls()
        self.class_index = class_mapping.index(obj.name)
        x0, y0, x1, y1 = obj.bndbox
        box_width = (x1 - x0) / image_width
        box_height = (y1 - y0) / image_height
        cx = x0 / image_width + box_width / 2.0
        cy = y0 / image_height + box_height / 2.0

        self.bndbox[0] = cx
        self.bndbox[1] = cy
        self.bndbox[2] = box_width
        self.bndbox[3] = box_height
        
        self.constrain_bndbox()

        return self

    def constrain_bndbox(self):
        cx, cy, w, h = self.bndbox
        assert w > 0.0, self.bndbox
        assert h > 0.0, self.bndbox
        
        x0 = cx - w / 2.0
        y0 = cy - h / 2.0
        
        if x0 < 0.0:
            x0 = 0.0
        if y0 < 0.0:
            y0 = 0.0
        
        if x0 > 1.0:
            x0 = 1.0
        if y0 > 1.0:
            y0 = 1.0
        
        x1 = x0 + w
        y1 = y0 + h
        
        if x1 > 1.0:
            x1 = 1.0
        if y1 > 1.0:
            y1 = 1.0
        
        w = x1 - x0
        h = y1 - y0
        
        cx = x0 + w / 2.0
        cy = y0 + h / 2.0
        
        self.bndbox = [cx, cy, w, h]
        assert self.bndbox_is_ok(), self.bndbox

    def bndbox_is_ok(self):
        cx, cy, w, h = self.bndbox
        
        x0 = cx - w / 2.0
        y0 = cy - h / 2.0
        
        if w <= 0.0 or h <= 0.0:
            return False
        if x0 < 0.0 or y0 < 0.0:
            return False
        return True

    def to_txt(self):
        string = str(self.class_index) + " "
        string += " ".join(map(self._format_bndbox_element, self.bndbox))
        string += "\n"
        return string

    @staticmethod
    def _format_bndbox_element(x):
        return "%0.6f" % x


class YoloFrame:
    def __init__(self):
        self.objects = []

    def write(self, path):
        contents = ""
        for obj in self.objects:
            contents += obj.to_txt()
        with open(path, 'w') as file:
            file.write(contents)

    @classmethod
    def from_pascal_voc(cls, frame, class_mapping: list):
        self = cls()
        for obj in frame.objects:
            self.objects.append(YoloObject.from_pascal_voc(obj, class_mapping, frame.width, frame.height))
        return self
