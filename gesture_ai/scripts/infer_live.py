import os
import cv2
import torch
import torchvision.transforms as transforms
from torchvision import models
from torch import nn

# === Config ===
MODEL_PATH = "/Users/ceylan/csse4011/csse4011Project/gesture_ai/models/stop_classifier.pth"
IMG_SIZE = 224
DEVICE = torch.device("mps" if torch.backends.mps.is_available() else "cuda" if torch.cuda.is_available() else "cpu")
CLASS_NAMES = ["none", "stop"]  # Must match training order

# === Load Model ===
model = models.mobilenet_v2(pretrained=False)
model.classifier[1] = nn.Linear(model.last_channel, 2)
model.load_state_dict(torch.load(MODEL_PATH, map_location=DEVICE))
model.eval().to(DEVICE)

# === Transform ===
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

# === Open Webcam ===
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Could not open webcam.")
    exit()

print("✅ Webcam open — press Q to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    input_img = transform(frame).unsqueeze(0).to(DEVICE)

    with torch.no_grad():
        output = model(input_img)
        _, pred = torch.max(output, 1)
        label = CLASS_NAMES[pred.item()]
        confidence = torch.softmax(output, dim=1)[0][pred].item()

    # === Display Result ===
    text = f"{label.upper()} ({confidence*100:.1f}%)"
    color = (0, 255, 0) if label == "stop" else (0, 0, 255)
    cv2.putText(frame, text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
    cv2.imshow("STOP Gesture Detector", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
