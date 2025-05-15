import cv2
import os

# === Path Setup ===
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
data_path = os.path.join(BASE_DIR, "data", "train")

# Define gesture categories
gestures = {
    "s": os.path.join(data_path, "static", "stop"),   # [s] = stop
    "n": os.path.join(data_path, "none"),             # [n] = none
    # You can later add: "g": go_path, "l": left_path, etc.
}

# Create directories if they don’t exist
for path in gestures.values():
    os.makedirs(path, exist_ok=True)

# === Start Webcam ===
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Error: Could not access webcam")
    exit()

print("👋 Press a key to record a gesture:")
print("[s] Stop | [n] None | [q] Quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Show live feed
    cv2.imshow("Gesture Capture", frame)

    # Wait for key press
    key = cv2.waitKey(1) & 0xFF
    if key != 255:
        print(f"🔤 Key pressed: {chr(key)} ({key})")

    # Quit key
    if key == ord("q"):
        break

    # Save image if valid key
    key_char = chr(key)
    if key_char in gestures:
        save_dir = gestures[key_char]
        gesture_name = os.path.basename(save_dir)
        img_name = f"{gesture_name}_{int(cv2.getTickCount())}.jpg"
        img_path = os.path.join(save_dir, img_name)
        cv2.imwrite(img_path, frame)
        print(f"✅ Image saved to: {img_path}")

# === Cleanup ===
cap.release()
cv2.destroyAllWindows()
