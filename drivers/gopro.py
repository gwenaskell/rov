from goprocam import GoProCamera, constants
import subprocess

goproCamera = GoProCamera.GoPro()
# goproCamera.pair(usepin=False)
goproCamera.infoCamera("model_name")
goproCamera.livestream("start")
goproCamera.video_settings("1080pSV")
# constants.Video.Resolution.R2kSV
goproCamera.video_settings("", fps="24")
goproCamera.gpControlSet(constants.Stream.WINDOW_SIZE, "4")
goproCamera.gpControlSet(constants.Stream.BIT_RATE, constants.Stream.BitRate.B1_2Mbps)
try:
    goproCamera.KeepAlive()
finally:
    goproCamera.livestream("stop")



# goproCamera.stream("udp://127.0.0.1:10000")

# goproCamera = GoProCamera.GoPro()
# goproCamera.webcamFOV(constants.Webcam.FOV.Wide)
# goproCamera.startWebcam("1080sv")
# # subprocess.Popen("ffplay -flags low_delay -fflags nobuffer -probesize 32 -analyzeduration 0 -f mpegts udp://:8554", shell=True)
# try:
#     goproCamera.KeepAlive()
# finally:
#     goproCamera.stopWebcam()
#     # goproCamera.livestream("stop")
