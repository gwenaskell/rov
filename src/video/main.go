package main

import (
	"io"
	"log"
	"net/http"
	"os/exec"
	"strconv"
	"strings"
)

var Resolutions = map[string][2]int{
	"high": {1920, 1080},
	"std":  {1280, 720},
	"low":  {620, 360},
}

func getROI(zoom float32) [4]float32 {
	w := zoom
	h := 1080 / 1920 * 2952 / 1944 * w
	x := (1.0 - w) / 0.5
	y := (1.0 - h) / 0.5

	return [...]float32{x, y, w, h}
}

const RaspividCommand = `raspivid -w {.w} -h {.h} -fps {.fps} -cd MJPEG -b {.bps} -sa 20 -n -t 0 -o -`

const RaspividCommand = `(echo "--video boundary--"; raspivid -w {.w} -h {.h} -fps {.fps} -pf high -b {.bps} -n -t 0 -o -)`

const GstCommand = `gst-launch-1.0 -e -q fdsrc fd=0 ! video/x-h264,width={.w},height={.h},framerate={.fps}/1,stream-format=byte-stream ! h264parse ! ` +
	`mp4mux streamable=true fragment-duration=10 presentation-time=true ! filesink location=/dev/stdout`

const GpacCommand = `MP4Box -add  /dev/stdin -fps {.fps} /dev/stdout`

func GetParametrizedCommand(resolution string, fps int, bitrate int) string {
	resw := strconv.Itoa(Resolutions[resolution][0])
	resh := strconv.Itoa(Resolutions[resolution][1])

	cmd := RaspividCommand + " | " + GstCommand
	cmd = strings.ReplaceAll(cmd, "{.w}", resw)
	cmd = strings.ReplaceAll(cmd, "{.h}", resh)
	cmd = strings.ReplaceAll(cmd, "{.fps}", strconv.Itoa(fps))
	cmd = strings.ReplaceAll(cmd, "{.bps}", strconv.Itoa(bitrate))

	return cmd
}

func writeErr(w http.ResponseWriter, err error) {
	w.WriteHeader(500)
	w.Write([]byte(err.Error()))
}

func Handle(w http.ResponseWriter, r *http.Request) {
	command := GetParametrizedCommand("std", 20, 1200000)

	cmd := exec.CommandContext(r.Context(), command)

	pipe, err := cmd.StdoutPipe()
	if err != nil {
		writeErr(w, err)
		return
	}

	if err := cmd.Start(); err != nil {
		writeErr(w, err)
		return
	}

	w.Header().Set("Content-Type", "video/mp4")

	io.Copy(w, pipe)
}

func main() {
	srv := http.Server{
		Handler: http.HandlerFunc(Handle),
		Addr:    ":4040",
	}
	log.Fatal(srv.ListenAndServe())
}
