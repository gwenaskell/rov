exit 1

ffmpeg -f mpegts -r 24 \
  -i udp://:8554 \
  -vaapi_device /dev/dri/renderD128 \
  -vf 'format=nv12,hwupload,scale_vaapi=w=1920:h=1080' \
  -c:v h264_vaapi -qp:v 26 -bf 0 -tune zerolatency -f mpegts \
  udp://127.0.0.1:10000


ffmpeg -f x11grab -s 1920x1080 -framerate 24 -i udp://:8554 \
  -vaapi_device /dev/dri/renderD128 \
  -vf 'format=nv12,hwupload,scale_vaapi=w=1920:h=1080' \
  -c:v h264_vaapi -qp:v 26 -bf 0 -tune zerolatency -f mpegts \
  udp://127.0.0.1:10000



sudo sysctl -w net.ipv4.ip_forward=1

sysctl net.ipv4.ip_forward

sudo iptables -t nat -A PREROUTING -i wlp0s20f3 -p udp --dport 8554 -j DNAT --to 192.168.253.100

sudo iptables -t nat -A POSTROUTING -o enp0s31f6 -p udp --dport 8554 -j SNAT --to-source 192.168.253.101

sudo iptables -t nat -A POSTROUTING -p udp --sport 8554 -j SNAT --to-source 10.5.5.100

## DROP
sudo iptables -t nat -D PREROUTING 1
sudo iptables -t nat -D POSTROUTING 1
