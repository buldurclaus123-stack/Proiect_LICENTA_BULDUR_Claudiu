import cv2
from ultralytics import YOLO
import math
import serial
import struct
import threading
import numpy as np
import subprocess
import sys


MODEL_PATH = "best.pt" 
LIDAR_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 128000  
CAMERA_INDEX = 0   

FPS = 30 
FOV_H_CAM = 48.7 


OFFSET_X_MM = 100.0 
OFFSET_Z_MM = 500.0 
OFFSET_Y_MM = 0.0   


SRT_URL = (
    "srt://192.168.90.201:8890?"
    "streamid=publish:camera-rgb:robot:disp!2026.upb&"
    "pkt_size=1316&"
    "passphrase=G1wgqw4zjiThI9bYzBC6OkIJ8IiYCpdWJaYFthODnfE="
)


command = [
    'ffmpeg',
    '-y',                 
    '-f', 'rawvideo',
    '-vcodec', 'rawvideo',
    '-pix_fmt', 'bgr24',
    '-s', '1280x720',     
    '-r', str(FPS),
    '-i', '-', 
    '-c:v', 'libx264',
    '-preset', 'ultrafast',
    '-tune', 'zerolatency',
    '-pix_fmt', 'yuv420p', 
    '-b:v', '1M',         
    '-g', '30', 
    '-f', 'mpegts',
    SRT_URL
]
 
CLASS_MAP = {0: "Masina", 1: "Persoana", 2: "Pisica", 3: "Porumbel"}
colors = {"Masina": (0, 0, 255), "Persoana": (0, 255, 0), "Pisica": (255, 0, 0), "Porumbel": (128, 0, 128)}


harta_lidar = np.zeros(360) 
running = True
scan_sync_event = threading.Event()

def lidar_thread():
    global harta_lidar, running, scan_sync_event
    try:
        ser = serial.Serial(LIDAR_PORT, BAUD_RATE, timeout=0.1)
        ser.write(b'\xA5\x60') 
        while running:
            if ser.read(1) != b'\xaa': continue
            if ser.read(1) != b'\x55': continue
            header_data = ser.read(8)
            if len(header_data) < 8: continue
            ct, lsn, fsa_raw, lsa_raw, cs = struct.unpack('<BBHHH', header_data)
            
            if ct & 0x01 == 1: 
                scan_sync_event.set() 
            
            if lsn == 0: continue
            raw_data = ser.read(lsn * 2)
            if len(raw_data) < lsn * 2: continue
            samples = struct.unpack('<' + 'H' * lsn, raw_data)
            
            angle_fsa = (fsa_raw >> 1) / 64.0
            angle_lsa = (lsa_raw >> 1) / 64.0
            diff_angle = (angle_lsa - angle_fsa + 360.0) % 360.0
            
            for i in range(lsn):
                dist_mm = samples[i] / 4.0
                if 150 < dist_mm < 10000:
                    angle_i = (diff_angle / (lsn - 1) if lsn > 1 else 0) * i + angle_fsa
                    angle_corr = math.degrees(math.atan(21.8 * (155.3 - dist_mm) / (155.3 * dist_mm)))
                    angle_final = int((angle_i + angle_corr) % 360)
                    harta_lidar[angle_final] = dist_mm
    except Exception as e: 
        pass 
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.write(b'\xA5\x65')
            ser.close()

threading.Thread(target=lidar_thread, daemon=True).start()


model = YOLO(MODEL_PATH)

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

W_PIXELS = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H_PIXELS = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

if W_PIXELS == 0 or H_PIXELS == 0:
    print("[EROARE] Camera nu a fost detectată! Verifică cablul USB.")
    sys.exit(1)

print(f"[INFO] Camera pornită cu rezoluția nativă: {W_PIXELS}x{H_PIXELS}")

F_PIXELS = (W_PIXELS / 2) / math.tan(math.radians(FOV_H_CAM / 2))

try:
    print("\n[INFO] Pornire conexiune FFmpeg SRT...")
    pipe = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=sys.stderr)
    print("[OK] FFmpeg conectat.")
except FileNotFoundError:
    print("ERROR: FFmpeg nu este instalat pe sistem.")
    sys.exit(1)

try:
    while True:
        scan_sync_event.wait(timeout=0.1)
        scan_sync_event.clear() 
        
        ret, frame = cap.read()
        if not ret: 
            print("[EROARE] S-a pierdut semnalul camerei.")
            break
        
        cx_cam = W_PIXELS // 2
        cy_cam = H_PIXELS // 2
        
        cv2.circle(frame, (cx_cam, cy_cam), 5, (0, 255, 255), -1)

        
        results = model.predict(frame, conf=0.4, device=0, verbose=False)
        
        sum_x, sum_y = 0, 0
        nr_obiecte = 0
        info_dreapta_sus = []
        
        for r in results:
            for box in r.boxes:
                x_p, y_p, wb, hb = map(float, box.xywh[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                cls = int(box.cls[0])
                nume_clasa = CLASS_MAP.get(cls, "Necunoscut")
                color = colors.get(nume_clasa, (255, 255, 255))
                
                
                sum_x += x_p
                sum_y += y_p
                nr_obiecte += 1

                dx = x_p - cx_cam 
                phi_rad = math.atan(dx / F_PIXELS)
                phi_deg = math.degrees(phi_rad)
                
                unghi_cautare_lidar = int((360 - phi_deg) % 360)
                fereastra = [harta_lidar[a % 360] for a in range(unghi_cautare_lidar - 5, unghi_cautare_lidar + 6)]
                valide = [v for v in fereastra if v > 150.0]
                
                if valide:
                    dist_L = np.median(valide)
                    dist_x = dist_L * math.cos(phi_rad) + OFFSET_X_MM
                    dist_y = dist_L * math.sin(phi_rad)
                    dist_reala_3d = math.sqrt(dist_x**2 + dist_y**2 + OFFSET_Z_MM**2)
                    dist_reala_m = round(dist_reala_3d / 1000.0, 2)
                else:
                    dist_reala_m = 0.0

                
                info_dreapta_sus.append(f"{nume_clasa} - Ang: {int(phi_deg)} deg, Distanta: {dist_reala_m}m")

                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.line(frame, (cx_cam, cy_cam), (int(x_p), int(y_p)), (200, 200, 200), 1)
                
                
                mid_x = (cx_cam + int(x_p)) // 2
                mid_y = (cy_cam + int(y_p)) // 2
                cv2.putText(frame, f"{int(phi_deg)} deg", (mid_x, mid_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                
                eticheta = f"{nume_clasa} - Distanta: {dist_reala_m}m"
                cv2.putText(frame, eticheta, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        
        if nr_obiecte > 0:
            p_ech_x = int(sum_x / nr_obiecte)
            p_ech_y = int(sum_y / nr_obiecte)
            cv2.circle(frame, (p_ech_x, p_ech_y), 8, (255, 100, 0), -1)
            
            y_offset = 30
            for text in info_dreapta_sus:
                cv2.putText(frame, text, (W_PIXELS - 320, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                y_offset += 25

       
        frame_server = cv2.resize(frame, (1280, 720))

        if pipe.poll() is not None:
            print("\n[EROARE] FFmpeg s-a oprit neașteptat.")
            break

        try:
            pipe.stdin.write(frame_server.tobytes())
            pipe.stdin.flush()
        except BrokenPipeError:
            print("\n[BROKEN PIPE] Conexiunea s-a întrerupt.")
            break

        
        cv2.imshow("Transmisie SRT (Local Preview)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

except KeyboardInterrupt:
    print("\nOprire...")
finally:
    running = False
    if 'cap' in locals(): cap.release()
    if 'pipe' in locals() and pipe.poll() is None:
        pipe.stdin.close()
        pipe.wait()
    cv2.destroyAllWindows()