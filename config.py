MODEL_PATH = "best.pt"
CAMERA_INDEX = 0
FPS = 30
FOV_H_CAM = 48.7


LIDAR_PORT = '/dev/ttyUSB0'
BAUD_RATE = 128000


OFFSET_X_MM = 100.0
OFFSET_Z_MM = 500.0
OFFSET_Y_MM = 0.0

SRT_URL = (
    "srt://192.168.90.201:8890?"
    "streamid=publish:camera-rgb:robot:disp!2026.upb&"
    "pkt_size=1316&"
    "passphrase=G1wgqw4zjiThI9bYzBC6OkIJ8IiYCpdWJaYFthODnfE="
)


CLASS_MAP = {0: "Masina", 1: "Persoana", 2: "Pisica", 3: "Porumbel"}
COLORS = {
    "Masina": (0, 0, 255), 
    "Persoana": (0, 255, 0), 
    "Pisica": (255, 0, 0), 
    "Porumbel": (128, 0, 128)
}