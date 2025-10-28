import argparse
import cv2
import os
import torch
import numpy as np
import matplotlib.cm as cm
from datetime import datetime
from depth_anything_v2.dpt import DepthAnythingV2

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Depth Anything V2 - Webcam')
    parser.add_argument('--input-size', type=int, default=518)
    parser.add_argument('--outdir', type=str, default='./vis_webcam_depth')
    parser.add_argument('--encoder', type=str, default='vitl', choices=['vits', 'vitb', 'vitl', 'vitg'])
    parser.add_argument('--pred-only', dest='pred_only', action='store_true', help='Only display the depth prediction')
    parser.add_argument('--grayscale', dest='grayscale', action='store_true', help='Render depth in grayscale')
    parser.add_argument('--record', action='store_true', help='Record output to video file')
    args = parser.parse_args()

    # Device setup
    DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

    # Model config
    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }

    choice = 'vits'
    print(f"[INFO] Loading model {choice} on {DEVICE}")
    model = DepthAnythingV2(**model_configs[choice])
    model.load_state_dict(torch.load(f'../models/depth_anything_v2_vits.pth', map_location='cpu'))
    model = model.to(DEVICE).eval()

    cmap = cm.get_cmap("Spectral_r")
    margin_width = 50

    os.makedirs(args.outdir, exist_ok=True)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot access webcam")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)

    # Confirm actual resolution (camera might not respect request)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] Camera resolution set to: {frame_width}x{frame_height}")

    # frame_rate = int(cap.get(cv2.CAP_PROP_FPS) or 30)
    frame_rate = 60
    # Prepare output writer if recording
    if args.record:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = os.path.join(args.outdir, f'depth_webcam_{timestamp}.mp4')
        output_width = frame_width if args.pred_only else frame_width * 2 + margin_width
        writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), frame_rate, (output_width, frame_height))
        print(f"[INFO] Recording to: {out_path}")
    else:
        writer = None

    print("[INFO] Starting webcam feed. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Inference
        with torch.no_grad():
            depth = model.infer_image(frame, args.input_size)

        # Normalize depth to [0, 1]
        depth = (depth - depth.min()) / (depth.max() - depth.min())
        depth_norm = depth.astype(np.float32)

        # Render depth map
        if args.grayscale:
            depth_uint8 = (depth_norm * 255).astype(np.uint8)
            depth_rgb = np.repeat(depth_uint8[..., np.newaxis], 3, axis=-1)
        else:
            depth_rgb = (cmap(depth_norm)[:, :, :3] * 255).astype(np.uint8)
            depth_rgb = depth_rgb[:, :, ::-1]  # RGB → BGR for OpenCV

        # Compose output
        if args.pred_only:
            output = depth_rgb
        else:
            spacer = np.ones((frame_height, margin_width, 3), dtype=np.uint8) * 255
            output = cv2.hconcat([frame, spacer, depth_rgb])

        # Show live
        cv2.imshow("DepthAnythingV2 - Webcam", output)

        # Record if needed
        if writer:
            writer.write(output)

        # Exit key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()

