import os
import numpy as np

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

class YOLODetector:
    def __init__(self, weights_path, conf=0.25, iou=0.45):
        if YOLO is None:
            raise RuntimeError("Ultralytics YOLO not available. Install 'ultralytics'.")
        if not os.path.exists(weights_path):
            raise FileNotFoundError(f"YOLO weights not found: {weights_path}")
        self.model = YOLO(weights_path)
        self.conf = conf
        self.iou = iou

    def infer(self, image_bgr):
        # returns list of dicts: {bbox:[x1,y1,x2,y2], score:float, label:str}
        res = self.model(image_bgr, conf=self.conf, iou=self.iou)[0]
        out = []
        if hasattr(res, 'boxes'):
            xyxy = res.boxes.xyxy.cpu().numpy().astype(int)
            confs = res.boxes.conf.cpu().numpy().tolist()
            clss = res.boxes.cls.cpu().numpy().astype(int).tolist()
            names = res.names if hasattr(res, 'names') else {i: f"class_{i}" for i in set(clss)}
            for (x1,y1,x2,y2), s, c in zip(xyxy, confs, clss):
                out.append(dict(bbox=[int(x1),int(y1),int(x2),int(y2)], score=float(s), label=str(names.get(c, f'class_{c}'))))
        return out
