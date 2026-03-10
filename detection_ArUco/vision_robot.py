import cv2
import numpy as np
import subprocess
import cv2.aruco as aruco

# Configuration ArUco
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

print("--- MODE TURBO ACTIVÉ ---")

def get_optimized_frame():
    # On demande une capture très rapide
    pipe = subprocess.Popen(['adb', 'exec-out', 'screencap', '-p'], stdout=subprocess.PIPE)
    img_data = pipe.stdout.read()
    pipe.terminate() # On ferme immédiatement pour libérer l'USB
    
    if not img_data:
        return None
        
    nparr = np.frombuffer(img_data, np.uint8)
    return cv2.imdecode(nparr, cv2.IMREAD_COLOR)

while True:
    frame = get_optimized_frame()
    
    if frame is None:
        continue

    # RÉDUCTION DRASTIQUE : 320x180 pixels 
    # (C'est le secret pour que l'Alcatel ne chauffe pas)
    small_frame = cv2.resize(frame, (320, 180))
    
    gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        aruco.drawDetectedMarkers(small_frame, corners, ids)
        print(f"ID détecté : {ids[0][0]}")

    # On affiche la petite image (plus rapide à dessiner sur ton écran)
    cv2.imshow("Robot-Vision-LowRes", small_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
