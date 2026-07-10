import cv2
from ultralytics import YOLO
import sys

# Konfigurasi model (Sesuaikan dengan path di PC/Jetson)
BUOY_MODEL_PATH = "/home/amv/models/KKI-25/buoy_v1.engine"
BOX_MODEL_PATH = "/home/amv/models/KKI-25/box_v1.engine"

def main():
    print("=== TEST KAMERA DAN MODEL YOLO ===")
    
    # Minta input index kamera
    up_idx = input("Masukkan index kamera Atas (misal: 0 atau /dev/video0): ")
    down_idx = input("Masukkan index kamera Bawah (misal: 2 atau /dev/video2): ")
    
    # Konversi ke int jika input berupa angka
    if up_idx.isdigit(): up_idx = int(up_idx)
    if down_idx.isdigit(): down_idx = int(down_idx)
        
    print(f"\nMencoba membuka Kamera Atas ({up_idx})...")
    cap_up = cv2.VideoCapture(up_idx)
    
    print(f"Mencoba membuka Kamera Bawah ({down_idx})...")
    cap_down = cv2.VideoCapture(down_idx)
    
    if not cap_up.isOpened():
        print("[-] GAGAL membuka Kamera Atas!")
    if not cap_down.isOpened():
        print("[-] GAGAL membuka Kamera Bawah!")
        
    print("\nMemuat model YOLO (Harap tunggu beberapa detik)...")
    try:
        buoy_model = YOLO(BUOY_MODEL_PATH, task='detect')
        box_model = YOLO(BOX_MODEL_PATH, task='detect')
        print("[+] Model berhasil dimuat!")
    except Exception as e:
        print(f"[-] Gagal memuat model: {e}")
        print("Pastikan path model di script ini sudah benar.")
        sys.exit(1)

    print("\n[INFO] Menampilkan video. Tekan 'q' untuk keluar.")
    
    while True:
        ret_up, frame_up = cap_up.read()
        ret_down, frame_down = cap_down.read()
        
        if ret_up:
            # Jalankan model Buoy pada Kamera Atas
            results_up = buoy_model.predict(source=frame_up, conf=0.15, verbose=False)
            annotated_up = results_up[0].plot()
            cv2.imshow("Kamera Atas (Deteksi Buoy)", annotated_up)
            
        if ret_down:
            # Jalankan model Box pada Kamera Bawah (sebagai contoh)
            results_down = box_model.predict(source=frame_down, conf=0.4, verbose=False)
            annotated_down = results_down[0].plot()
            cv2.imshow("Kamera Bawah (Deteksi Box)", annotated_down)
            
        if not ret_up and not ret_down:
            print("Kamera terputus.")
            break
            
        # Keluar jika tombol 'q' ditekan
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap_up.release()
    cap_down.release()
    cv2.destroyAllWindows()
    print("Selesai.")

if __name__ == "__main__":
    main()
