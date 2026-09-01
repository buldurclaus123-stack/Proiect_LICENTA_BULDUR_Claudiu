import serial
import struct
import threading
import math
import numpy as np

class LidarHandler:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.harta_lidar = np.zeros(360)
        self.running = False
        self.scan_sync_event = threading.Event()
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._lidar_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def wait_for_scan(self, timeout=0.1):
        self.scan_sync_event.wait(timeout=timeout)
        self.scan_sync_event.clear()

    def get_map(self):
        return self.harta_lidar

    def _lidar_loop(self):
        try:
            ser = serial.Serial(self.port, self.baud_rate, timeout=0.1)
            ser.write(b'\xA5\x60') 
            
            while self.running:
                if ser.read(1) != b'\xaa': continue
                if ser.read(1) != b'\x55': continue
                
                header_data = ser.read(8)
                if len(header_data) < 8: continue
                
                ct, lsn, fsa_raw, lsa_raw, cs = struct.unpack('<BBHHH', header_data)
                
                if ct & 0x01 == 1: 
                    self.scan_sync_event.set() 
                
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
                        self.harta_lidar[angle_final] = dist_mm
        except Exception as e:
            pass
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.write(b'\xA5\x65')
                ser.close()