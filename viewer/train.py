from ultralytics import YOLO

model = YOLO("yolo11n.yaml").to("mps")
results = model.train(data="./datasets/data.yaml", epochs=10, imgsz=640)