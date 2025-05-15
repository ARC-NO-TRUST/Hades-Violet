import os
import cv2
import random
import numpy as np
from glob import glob

# === CONFIG ===
AUG_PER_IMAGE = 5  # how many augmentations per input image
DATASETS = {
    "stop": "gesture_ai/data/train/static/stop",
    "none": "gesture_ai/data/train/none"
}
EXTENSIONS = [".png", ".jpg", ".jpeg"]

# === AUGMENTATION FUNCTIONS ===
def flip(img):
    return cv2.flip(img, 1)

def brighten(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv[..., 2] = np.clip(hsv[..., 2] * random.uniform(0.7, 1.3), 0, 255)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

def rotate(img):
    h, w = img.shape[:2]
    angle = random.uniform(-10, 10)
    mat = cv2.getRotationMatrix2D((w/2, h/2), angle, 1)
    return cv2.warpAffine(img, mat, (w, h))

def crop(img):
    h, w = img.shape[:2]
    scale = random.uniform(0.85, 1.0)
    new_w, new_h = int(w * scale), int(h * scale)
    x = random.randint(0, w - new_w)
    y = random.randint(0, h - new_h)
    cropped = img[y:y+new_h, x:x+new_w]
    return cv2.resize(cropped, (w, h))

def noise(img):
    return cv2.add(img, np.random.normal(0, 10, img.shape).astype(np.uint8))

AUG_FUNCTIONS = [flip, brighten, rotate, crop, noise]

# === AUGMENTATION LOOP ===
for label, folder in DATASETS.items():
    images = sorted([
        f for f in glob(os.path.join(folder, "*"))
        if os.path.splitext(f)[1].lower() in EXTENSIONS and "_aug_" not in f
    ])

    count = 0
    for path in images:
        img = cv2.imread(path)
        name = os.path.splitext(os.path.basename(path))[0]

        for i in range(1, AUG_PER_IMAGE + 1):
            aug = img.copy()
            funcs = random.sample(AUG_FUNCTIONS, k=random.randint(2, 4))
            for func in funcs:
                aug = func(aug)

            new_name = f"{name}_aug_{i:02d}.jpg"
            cv2.imwrite(os.path.join(folder, new_name), aug)
            count += 1

    print(f"✅ {label.upper()}: Generated {count} new augmented images.")
