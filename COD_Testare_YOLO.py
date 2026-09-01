import cv2
from ultralytics import YOLO
import math
import csv

#PENTRU HOMPLEX :DACA TESTATI PROGRAMUL VA ROG SA INLOCUITI MODEL_PATH SI video_path cu cele din computerul dumneavoastra.
MODEL_PATH = r"C:\Users\claud\Desktop\Proiect_Varianta 1\runs\detect\train\weights\best.pt"
model = YOLO(MODEL_PATH)
video_path = r"C:\Users\claud\Desktop\Proiect_Varianta 1\Incercare4.mp4"
cap = cv2.VideoCapture(video_path)


PIXELS_PER_METER = 100 


csv_file = open('raport_final_complet.csv', mode='w', newline='')
writer = csv.writer(csv_file)
writer.writerow([
    'Cadru', 'Clasa_Obiect', 
    'X_Obj_m', 'Y_Obj_m', 
    'X_Echilibru_m', 'Y_Echilibru_m', 
    'Dist_fata_de_Echilibru_m', 
    'Unghi_fata_de_Centru_Cam_Deg'
])

CLASS_MAP = {0: "Masina", 1: "Persoana", 2: "Pisica", 3: "Porumbel"}
colors = {"Masina": (0, 0, 255), "Persoana": (0, 255, 0), "Pisica": (255, 0, 0), "Porumbel": (128, 0, 128)}

frame_count = 0
while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    frame_count += 1
    
  
    h_img, w_img, _ = frame.shape
    cx_cam, cy_cam = w_img // 2, h_img // 2
   
    cv2.circle(frame, (cx_cam, cy_cam), 5, (0, 255, 255), -1) 

    results = model.predict(frame, conf=0.3, verbose=False, device='cpu')
    
    detectii_cadru = []
    sum_x, sum_y = 0, 0

  
    for r in results:
        for box in r.boxes:
            x_c, y_c, w, h = map(float, box.xywh[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            
            if cls in CLASS_MAP:
                name = CLASS_MAP[cls]
                detectii_cadru.append({
                    "pos": (x_c, y_c), 
                    "name": name, 
                    "rect": (x1, y1, x2, y2), 
                    "conf": conf
                })
                sum_x += x_c
                sum_y += y_c

   
    nr_obiecte = len(detectii_cadru)
    if nr_obiecte > 0:
      
        p_ech_x = sum_x / nr_obiecte
        p_ech_y = sum_y / nr_obiecte
        cv2.circle(frame, (int(p_ech_x), int(p_ech_y)), 8, (255, 100, 0), -1)

        for obj in detectii_cadru:
            ox, oy = obj["pos"]
            x1, y1, x2, y2 = obj["rect"]
            color = colors[obj["name"]]
            
           
        
            cv2.line(frame, (cx_cam, cy_cam), (int(ox), int(oy)), (200, 200, 200), 1)
            
          
            dx = ox - cx_cam
            dy = oy - cy_cam
           
            unghi_rad = math.atan2(-dy, dx)
            unghi_deg = math.degrees(unghi_rad)

           
            dist_ech_m = math.sqrt((ox - p_ech_x)**2 + (oy - p_ech_y)**2) / PIXELS_PER_METER
            ox_m, oy_m = ox / PIXELS_PER_METER, oy / PIXELS_PER_METER
            cx_ech_m, cy_ech_m = p_ech_x / PIXELS_PER_METER, p_ech_y / PIXELS_PER_METER

           
            writer.writerow([
                frame_count, obj["name"], 
                round(ox_m, 2), round(oy_m, 2), 
                round(cx_ech_m, 2), round(cy_ech_m, 2), 
                round(dist_ech_m, 2), round(unghi_deg, 2)
            ])

            
          
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{obj['name']}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
           
            cv2.putText(frame, f"Ang: {int(unghi_deg)}deg", (x1, y2 + 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            cv2.putText(frame, f"C({int(ox)},{int(oy)})", (int(ox)+5, int(oy)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
           
            cv2.line(frame, (int(ox), int(oy)), (int(p_ech_x), int(p_ech_y)), (255, 255, 255), 1)

    cv2.imshow("Licenta - Analiza Completa: Metri, Echilibru, Unghi", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

csv_file.close()
cap.release()
cv2.destroyAllWindows()