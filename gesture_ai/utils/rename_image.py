import os
import re

# Folder path
TARGET_DIR = "/Users/ceylan/csse4011/csse4011Project/gesture_ai/data/train/static/stop"
EXTENSIONS = [".png", ".jpg", ".jpeg"]

# Regex to match already renamed files (e.g., stop_001.png)
stop_pattern = re.compile(r"^stop_\d{3}\.png$")

# Collect files that are not already properly named
files_to_rename = [
    f for f in sorted(os.listdir(TARGET_DIR))
    if os.path.splitext(f)[1].lower() in EXTENSIONS and not stop_pattern.match(f)
]

# Count how many already-named images exist to continue numbering
existing_stops = [
    f for f in os.listdir(TARGET_DIR)
    if stop_pattern.match(f)
]
starting_index = len(existing_stops) + 1

# Rename new files safely
for idx, filename in enumerate(files_to_rename, start=starting_index):
    new_name = f"stop_{idx:03d}.png"
    src_path = os.path.join(TARGET_DIR, filename)
    dst_path = os.path.join(TARGET_DIR, new_name)
    os.rename(src_path, dst_path)

print(f"✅ Renamed {len(files_to_rename)} new images safely starting from index {starting_index}.")
