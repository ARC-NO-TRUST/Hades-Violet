import os
import torch
import torchvision
import matplotlib.pyplot as plt
from torchvision import datasets, transforms, models
from torch.utils.data import random_split, DataLoader
from torch import nn, optim

# 🔧 Paths
STOP_DIR = "/Users/ceylan/csse4011/csse4011Project/gesture_ai/data/train/static/stop"
NONE_DIR = "/Users/ceylan/csse4011/csse4011Project/gesture_ai/data/train/none"
DATA_ROOT = "/Users/ceylan/csse4011/csse4011Project/gesture_ai/data/train/static_binary"
MODEL_PATH = "/Users/ceylan/csse4011/csse4011Project/gesture_ai/models/stop_classifier.pth"

# Step 1: Prepare a folder for ImageFolder structure
os.makedirs(DATA_ROOT, exist_ok=True)
os.symlink(STOP_DIR, os.path.join(DATA_ROOT, "stop"), target_is_directory=True)
os.symlink(NONE_DIR, os.path.join(DATA_ROOT, "none"), target_is_directory=True)

# ✅ Training Config
NUM_CLASSES = 2
BATCH_SIZE = 16
IMG_SIZE = 224
EPOCHS = 10
LR = 0.001
DEVICE = torch.device("mps" if torch.backends.mps.is_available() else "cuda" if torch.cuda.is_available() else "cpu")

# ✅ Transforms
transform = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.RandomHorizontalFlip(),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

# ✅ Load Dataset
dataset = datasets.ImageFolder(DATA_ROOT, transform=transform)
class_names = dataset.classes
train_size = int(0.8 * len(dataset))
val_size = len(dataset) - train_size
train_ds, val_ds = random_split(dataset, [train_size, val_size])
train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE)

# ✅ Model
model = models.mobilenet_v2(pretrained=True)
model.classifier[1] = nn.Linear(model.last_channel, NUM_CLASSES)
model.to(DEVICE)

criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=LR)

train_acc_hist, val_acc_hist = [], []
train_loss_hist, val_loss_hist = [], []

print(f"🔧 Training on: {DEVICE} | Classes: {class_names}")

# ✅ Training Loop
for epoch in range(EPOCHS):
    model.train()
    total, correct, loss_sum = 0, 0, 0

    for imgs, labels in train_loader:
        imgs, labels = imgs.to(DEVICE), labels.to(DEVICE)
        optimizer.zero_grad()
        outputs = model(imgs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        loss_sum += loss.item()
        correct += (outputs.argmax(1) == labels).sum().item()
        total += labels.size(0)

    train_loss = loss_sum / len(train_loader)
    train_acc = correct / total

    # ✅ Validation
    model.eval()
    val_loss, val_correct, val_total = 0, 0, 0
    with torch.no_grad():
        for imgs, labels in val_loader:
            imgs, labels = imgs.to(DEVICE), labels.to(DEVICE)
            outputs = model(imgs)
            val_loss += criterion(outputs, labels).item()
            val_correct += (outputs.argmax(1) == labels).sum().item()
            val_total += labels.size(0)

    val_loss /= len(val_loader)
    val_acc = val_correct / val_total

    train_acc_hist.append(train_acc)
    val_acc_hist.append(val_acc)
    train_loss_hist.append(train_loss)
    val_loss_hist.append(val_loss)

    print(f"[Epoch {epoch+1}/{EPOCHS}] "
          f"Train Acc: {train_acc:.4f}, Val Acc: {val_acc:.4f}")

# ✅ Save model
torch.save(model.state_dict(), MODEL_PATH)
print(f"✅ Model saved to: {MODEL_PATH}")

# ✅ Plot accuracy and loss
plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.plot(train_loss_hist, label='Train Loss')
plt.plot(val_loss_hist, label='Val Loss')
plt.title("Loss")
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(train_acc_hist, label='Train Acc')
plt.plot(val_acc_hist, label='Val Acc')
plt.title("Accuracy")
plt.legend()
plt.tight_layout()
plt.show()
