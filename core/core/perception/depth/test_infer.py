import cv2
import torch
import numpy as np
import threading
import time
from queue import Queue
from depth_anything_v2.dpt import DepthAnythingV2

# === CONFIG ===
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
ENCODER = 'vits'  # or 'vitb', 'vitl', 'vitg'
MODEL_PATH = '../models/depth_anything_v2_vits.pth'
INPUT_RESOLUTION = (384, 288)  # Width x Height
FRAME_SKIP = 2  # Only process every N frames
USE_HALF = DEVICE == 'cuda'
SHOW_EVERY_N_FRAMES = 2  # Visualization throttle

# === LOAD MODEL ===
model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
}

print(f"Loading model: {ENCODER} on {DEVICE}")
model = DepthAnythingV2(**model_configs[ENCODER])
model.load_state_dict(torch.load(MODEL_PATH, map_location='cpu'))
model = model.to(DEVICE).eval()

# Use half precision if on CUDA
if USE_HALF:
    model = model.half()

# Use torch.compile if available
try:
    model = torch.compile(model)
    print("Model compiled with torch.compile()")
except Exception as e:
    print("torch.compile() failed or not supported, continuing without it.")

# === THREADING: Webcam capture ===
frame_queue = Queue(maxsize=1)

def capture_thread():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot access webcam")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        if frame_queue.full():
            continue  # Drop frame if not consumed

        frame_queue.put(frame)

threading.Thread(target=capture_thread, daemon=True).start()

# === INFERENCE LOOP ===
frame_count = 0
print("Running... Press 'q' to quit.")

while True:
    if frame_queue.empty():
        time.sleep(0.01)
        continue

    frame = frame_queue.get()
    frame_count += 1

    # Skip frames to maintain FPS
    if frame_count % FRAME_SKIP != 0:
        continue

    # Resize input
    input_frame = cv2.resize(frame, INPUT_RESOLUTION)

    # Prepare input tensor
    input_tensor = torch.from_numpy(input_frame).permute(2, 0, 1).unsqueeze(0).float() / 255.0
    input_tensor = input_tensor.to(DEVICE)
    if USE_HALF:
        input_tensor = input_tensor.half()

    # Inference
    start_time = time.time()
    with torch.no_grad():
        depth = model.infer_image(input_tensor)  # Returns numpy array
    inference_time = time.time() - start_time
    print(f"Inference time: {inference_time:.3f}s")

    # Normalize and visualize depth
    depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
    depth_vis = depth_vis.astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_MAGMA)
    depth_color = cv2.resize(depth_color, (frame.shape[1], frame.shape[0]))

    # Show every N frames to avoid GUI lag
    if frame_count % SHOW_EVERY_N_FRAMES == 0:
        combined = np.hstack((frame, depth_color))
        cv2.imshow("Webcam RGB (Left) | Depth (Right)", combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

