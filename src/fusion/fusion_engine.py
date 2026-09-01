import math
import numpy as np

class FusionEngine:
    def __init__(self, offset_x, offset_z, f_pixels, cx_cam):
        self.offset_x = offset_x
        self.offset_z = offset_z
        self.f_pixels = f_pixels
        self.cx_cam = cx_cam

    def calculate_distance(self, x_p, harta_lidar):
        dx = x_p - self.cx_cam 
        phi_rad = math.atan(dx / self.f_pixels)
        phi_deg = math.degrees(phi_rad)
        
        unghi_cautare_lidar = int((360 - phi_deg) % 360)
        fereastra = [harta_lidar[a % 360] for a in range(unghi_cautare_lidar - 5, unghi_cautare_lidar + 6)]
        valide = [v for v in fereastra if v > 150.0]
        
        if valide:
            dist_L = np.median(valide)
            dist_x = dist_L * math.cos(phi_rad) + self.offset_x
            dist_y = dist_L * math.sin(phi_rad)
            dist_reala_3d = math.sqrt(dist_x**2 + dist_y**2 + self.offset_z**2)
            dist_reala_m = round(dist_reala_3d / 1000.0, 2)
            return dist_reala_m, int(phi_deg)
            
        return 0.0, int(phi_deg)