import * as React from "react"
import Switch from "@mui/material/Switch"
import CycloneIcon from "@mui/icons-material/Cyclone"
import PauseCircleOutlineIcon from "@mui/icons-material/PauseCircleOutline"
import PlayCircleFilledIcon from "@mui/icons-material/PlayCircleFilled"
import GpsOff from "@mui/icons-material/GpsOff"
import { Container, IconButton } from "@mui/material"
import { setEngineState } from "../../store/controlsSlice"
import { useDispatch, useSelector } from "../../store/hooks"
import { styled } from "@mui/material/styles"
import { useState, useEffect } from "react"

const EnginesState = () => {
  const dispatch = useDispatch()

  const MaterialUISwitch = styled(Switch)(({ theme }) => ({
    width: 62,
    height: 34,
    padding: 7,
    "& .MuiSwitch-switchBase": {
      margin: 1,
      padding: 0,
      transform: "translateX(6px)",
      "&.Mui-checked": {
        color: "#fff",
        transform: "translateX(22px)",
        "& .MuiSwitch-thumb:before": {
          backgroundImage: `url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" height="20" width="20" viewBox="0 0 24 24"><path fill="${encodeURIComponent(
            "#fff"
          )}" d="M22 7.47V5.35C20.05 4.77 16.56 4 12 4c-2.15 0-4.11.86-5.54 2.24.13-.85.4-2.4 1.01-4.24H5.35C4.77 3.95 4 7.44 4 12c0 2.15.86 4.11 2.24 5.54-.85-.14-2.4-.4-4.24-1.01v2.12C3.95 19.23 7.44 20 12 20c2.15 0 4.11-.86 5.54-2.24-.14.85-.4 2.4-1.01 4.24h2.12c.58-1.95 1.35-5.44 1.35-10 0-2.15-.86-4.11-2.24-5.54.85.13 2.4.4 4.24 1.01zM12 18c-3.31 0-6-2.69-6-6s2.69-6 6-6 6 2.69 6 6-2.69 6-6 6z"/></svg>')`,
        },
        "& + .MuiSwitch-track": {
          opacity: 1,
          backgroundColor:
            theme.palette.mode === "dark" ? "#8796A5" : "#aab4be",
        },
      },
    },
    "& .MuiSwitch-thumb": {
      backgroundColor: theme.palette.mode === "dark" ? "#003892" : "#001e3c",
      width: 32,
      height: 32,
      "&:before": {
        content: "''",
        position: "absolute",
        width: "100%",
        height: "100%",
        left: 0,
        top: 0,
        backgroundRepeat: "no-repeat",
        backgroundPosition: "center",
        backgroundImage: `url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" height="20" width="20" viewBox="0 0 24 24"><path fill="${encodeURIComponent(
          "#fff"
        )}" d="M20.94 11c-.46-4.17-3.77-7.48-7.94-7.94V1h-2v2.06c-1.13.12-2.19.46-3.16.97l1.5 1.5C10.16 5.19 11.06 5 12 5c3.87 0 7 3.13 7 7 0 .94-.19 1.84-.52 2.65l1.5 1.5c.5-.96.84-2.02.97-3.15H23v-2h-2.06zM3 4.27l2.04 2.04C3.97 7.62 3.25 9.23 3.06 11H1v2h2.06c.46 4.17 3.77 7.48 7.94 7.94V23h2v-2.06c1.77-.2 3.38-.91 4.69-1.98L19.73 21 21 19.73 4.27 3 3 4.27zm13.27 13.27C15.09 18.45 13.61 19 12 19c-3.87 0-7-3.13-7-7 0-1.61.55-3.09 1.46-4.27l9.81 9.81z"/></svg>')`,
      },
    },
    "& .MuiSwitch-track": {
      opacity: 1,
      backgroundColor: theme.palette.mode === "dark" ? "#8796A5" : "#aab4be",
      borderRadius: 20 / 2,
    },
  }))
  const handleSwitch = (event: React.ChangeEvent<HTMLInputElement>) => {
    if (event.target.checked) {
      dispatch(setEngineState("running"))
    } else {
      dispatch(setEngineState("off"))
    }
  }

  const eState = useSelector((state) => state.controls.enginesState)

  const handleClick = () => {
    if (eState === "running") {
      dispatch(setEngineState("paused"))
    } else if (eState === "paused") {
      dispatch(setEngineState("running"))
    }

    fetch("http://" + window.location.hostname + ":8000/engines", {
      method: "PUT",
      body: JSON.stringify({
        on: eState !== "off",
        paused: eState === "paused",
      }),
      headers: {
        Accept: "application/json",
        "Content-Type": "application/json",
      },
    }).catch((err) => {
      console.log(err.message)
    })
  }

  return (
    <Container>
      <MaterialUISwitch onChange={handleSwitch} />
      <IconButton
        disabled={eState === "off"}
        onClick={handleClick}
        color={eState === "paused" ? "warning" : "primary"}
      >
        {eState === "paused" ? (
          <PauseCircleOutlineIcon />
        ) : (
          <PlayCircleFilledIcon />
        )}
      </IconButton>
    </Container>
  )
}

export default EnginesState
