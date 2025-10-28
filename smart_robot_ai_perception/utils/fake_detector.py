import numpy as np

class FakeDetector:
    def __init__(self):
        pass

    def infer(self, image_bgr):
        # Create a single box in the image center as a placeholder detection
        h, w = image_bgr.shape[:2]
        bw, bh = w//6, h//6
        cx, cy = w//2, h//2
        x1, y1 = max(0, cx - bw//2), max(0, cy - bh//2)
        x2, y2 = min(w-1, cx + bw//2), min(h-1, cy + bh//2)
        return [dict(bbox=[x1,y1,x2,y2], score=0.5, label="target")]
