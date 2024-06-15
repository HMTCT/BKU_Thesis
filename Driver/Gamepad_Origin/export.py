from ultralytics import YOLO

# Load a model
model = YOLO('/home/dofarm/Desktop/BKU_Thesis/Driver/Jetson_python_driver/best_model.pt')  # load an official model

# Export the model
model.export(format='onnx')
