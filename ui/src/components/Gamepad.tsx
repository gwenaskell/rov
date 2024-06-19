import { Chip } from "@mui/material"
import Gamepad from "../gamepad"
import CancelIcon from "@mui/icons-material/Cancel"
import DoneIcon from "@mui/icons-material/Done"
import {
  setAxis,
  setButton,
  setConnected,
  AxisType,
  ButtonType,
} from "../store/gamepadSlice"
import { useDispatch, useSelector } from "../store/hooks"
import WS from "../libs/WebSocket"
import { store } from "../store/store"

// https://dev.to/whoisryosuke/adding-game-controller-input-to-react-5d13

const layout = {
  buttons: [
    "A",
    "B",
    "Y",
    "X",
    "LB",
    "RB",
    "LT",
    "RT",
    "LS",
    "RS",
    "Back",
    "Start",
    "DPadUp",
    "DPadDown",
    "DPadLeft",
    "DPadRight",
    "Unknown1",
    "Unknown2",
    "Unknown3",
  ],
  axis: [
    "leftY",
    "-leftX",
    "padY",
    "-padX",
    "rightY",
    "-rightX",
    "Unknown1",
    "Unknown2",
  ],
  //buttonAxis: [null, null, null, null, null, null, 'LeftTrigger', 'RightTrigger']
}

function mapAxisName(axis: string): AxisType | "" {
  switch (axis) {
    case "leftX":
    case "leftY":
    case "rightX":
    case "rightY":
    case "padX":
    case "padY":
      return axis
  }
  return ""
}

function mapButtonName(button: string): ButtonType | "" {
  switch (button) {
    case "LT":
      return "L2"
    case "LB":
      return "L"
    case "RT":
      return "R2"
    case "RB":
      return "R"
    case "A":
    case "B":
    case "X":
    case "Y":
      return button
    case "DPadLeft":
      return "stickL"
    case "DPadRight":
      return "stickR"
  }
  return ""
}

export default function Controller() {
  const dispatch = useDispatch()

  const connected = useSelector((state) => state.gamepad.connected)

  const buttonChangeHandler = (buttonName: string, down: boolean) => {
    console.log("button", buttonName, down)
    let button = mapButtonName(buttonName)
    if (!button) {
      return
    }

    dispatch(
      setButton({
        axis: button,
        down: down,
      })
    )

    let state = store.getState()

    WS.send(
      JSON.stringify({ tm_ms: Date.now(), buttons: state.gamepad.buttons })
    )
  }

  const axisChangeHandler = (
    axisName: string,
    value: number,
    _previousValue: number
  ) => {
    console.log("axis", axisName, value)
    let axis = mapAxisName(axisName)

    if (axis === "") {
      return
    }

    value = Math.round(value * 20) * 5

    dispatch(
      setAxis({
        axis: axis,
        value: value,
      })
    )

    let state = store.getState()

    WS.send(JSON.stringify({ tm_ms: Date.now(), sticks: state.gamepad.sticks }))
  }

  const onConnect = () => {
    dispatch(setConnected(true))

    WS.setGMConnection(true)
  }

  const onDisconnect = () => {
    dispatch(setConnected(false))

    WS.setGMConnection(false)
  }

  return (
    <Gamepad
      layout={layout as any}
      onConnect={onConnect}
      onDisconnect={onDisconnect}
      onButtonChange={buttonChangeHandler}
      onAxisChange={axisChangeHandler}
      deadZone={0.01}
    >
      <Chip
        label={connected ? "Gamepad connected" : "Gamepad disconnected"}
        icon={
          connected ? (
            <DoneIcon sx={{ fontSize: 20 }} />
          ) : (
            <CancelIcon sx={{ fontSize: 20 }} />
          )
        }
        variant="outlined"
        color={connected ? "success" : "error"}
        sx={{ fontSize: 12 }}
      />
    </Gamepad>
  )
}
