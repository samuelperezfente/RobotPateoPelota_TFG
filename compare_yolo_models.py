import os
import cv2
import time
import numpy as np
from ultralytics import YOLO

# Modelos

MODEL_DIR = "/ros2_ws/src/go2_ros2_sdk/src/yolo/yolobot_recognition/scripts"

# Modelos (usando os.path.join para no repetir la ruta)
models = {
    "ground_truth": YOLO(os.path.join(MODEL_DIR, "bestv6.pt")),
    "my_yolov5nu": YOLO(os.path.join(MODEL_DIR, "my_yolov5nu.pt")),
    "my_yolov5su": YOLO(os.path.join(MODEL_DIR, "my_yolov5su.pt")),
    "my_yolov8n": YOLO(os.path.join(MODEL_DIR, "my_yolov8n.pt")),
    "my_yolov8s": YOLO(os.path.join(MODEL_DIR, "my_yolov8s.pt")),
    "my_yolov11n": YOLO(os.path.join(MODEL_DIR, "my_yolov11n.pt")),
    "my_yolov11s": YOLO(os.path.join(MODEL_DIR, "my_yolov11s.pt")),
}

# # Modelos (usando os.path.join para no repetir la ruta)
# models = {
#     "ground_truth": YOLO(os.path.join(MODEL_DIR, "my_yolov11s.pt")),
#     "my_yolov5nu": YOLO(os.path.join(MODEL_DIR, "my_yolov5nu.pt")),
#     "my_yolov5su": YOLO(os.path.join(MODEL_DIR, "my_yolov5su.pt")),
#     "my_yolov8n": YOLO(os.path.join(MODEL_DIR, "my_yolov8n.pt")),
#     "my_yolov8s": YOLO(os.path.join(MODEL_DIR, "my_yolov8s.pt")),
#     "my_yolov11n": YOLO(os.path.join(MODEL_DIR, "my_yolov11n.pt")),
#     "my_yolov11s": YOLO(os.path.join(MODEL_DIR, "my_yolov11s.pt")),
# }

# # Modelos (usando os.path.join para no repetir la ruta)
# models = {
#     "ground_truth": YOLO(os.path.join(MODEL_DIR, "bestv6.pt")),
#     "my_yolov11n": YOLO(os.path.join(MODEL_DIR, "my_yolov11n.pt")),
# }

# Dataset
image_dir = "/ros2_ws/yolo_frames"
image_files = sorted(os.listdir(image_dir))

def iou(box1, box2):
    """Calcula IoU entre dos bboxes [xmin, ymin, xmax, ymax]."""
    xA = max(box1[0], box2[0])
    yA = max(box1[1], box2[1])
    xB = min(box1[2], box2[2])
    yB = min(box1[3], box2[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    box1Area = (box1[2]-box1[0]) * (box1[3]-box1[1])
    box2Area = (box2[2]-box2[0]) * (box2[3]-box2[1])
    unionArea = box1Area + box2Area - interArea
    return interArea / unionArea if unionArea > 0 else 0

def get_detections(results):
    detections = []
    for r in results:
        for b in r.boxes:
            bbox = b.xyxy[0].cpu().numpy()
            cls = int(b.cls)
            detections.append((cls, bbox))
    return detections

# Métricas por modelo
metrics = {name: {"tp": 0, "fp": 0, "fn": 0, "times": []} for name in models if name != "ground_truth"}

for fname in image_files:
    img = cv2.imread(os.path.join(image_dir, fname))

    # Ground truth detections
    gt_results = models["ground_truth"](img)
    gt_dets = get_detections(gt_results)

    for name, model in models.items():
        if name == "ground_truth":
            continue
        
        start = time.time()
        results = model(img)
        elapsed = time.time() - start
        metrics[name]["times"].append(elapsed)

        dets = get_detections(results)

        matched_gt = set()
        for cls, bbox in dets:
            match_found = False
            for j, (gt_cls, gt_bbox) in enumerate(gt_dets):
                if gt_cls == cls and iou(bbox, gt_bbox) >= 0.5 and j not in matched_gt:
                    metrics[name]["tp"] += 1
                    matched_gt.add(j)
                    match_found = True
                    break
            if not match_found:
                metrics[name]["fp"] += 1
        
        # FNs = ground truth no detectados
        metrics[name]["fn"] += len(gt_dets) - len(matched_gt)

# Mostrar resultados
for name, m in metrics.items():
    precision = m["tp"] / (m["tp"] + m["fp"]) if (m["tp"] + m["fp"]) > 0 else 0
    recall = m["tp"] / (m["tp"] + m["fn"]) if (m["tp"] + m["fn"]) > 0 else 0
    avg_time = np.mean(m["times"])
    print(f"Modelo {name}: Precision={precision:.3f}, Recall={recall:.3f}, Tiempo medio={avg_time:.3f}s")
