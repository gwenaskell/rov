import Controller from "./Gamepad";
import { Box, Grid } from "@mui/material";
import { useSelector } from "../store/hooks";

// https://www.iana.org/assignments/media-types/media-types.xhtml#video

export default function Header() {
  const muted = useSelector((state) => state.controls.mute);

  return (
    <Box className="videoContainer" sx={{ zIndex: -100 }}>
      <video
        id="player"
        muted={muted}
        autoPlay
        playsInline
        disablePictureInPicture
      >
        <source src="nautilus.mp4" type="video/mp4" />
      </video>
    </Box>
  );
}
