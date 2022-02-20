import time
import socket

import cv2
from goprocam import GoProCamera, constants

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t=time.time()

gopro = GoProCamera.GoPro()
gopro.livestream("start")
gopro.video_settings(res='1080pSV')
gopro.video_settings(res='1080pSV', fps='24')

gopro.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.R720)

cap = cv2.VideoCapture("udp://10.5.5.9:8554", cv2.CAP_FFMPEG)
while True:
    nmat, frame = cap.read()
    cv2.imshow("GoPro OpenCV", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if time.time() - t >= 2.5:
        sock.sendto("_GPHD_:0:0:2:0.000000\n".encode(), ("10.5.5.9", 8554))
        t=time()
# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()