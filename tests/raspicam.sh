# on the computer
netcat -l -p 5000 | mplayer -fps 60 -cache 2048 -

# on the raspberry
raspivid -t 0 -w 1280 -h 720 -o - | nc 192.168.253.102 5000