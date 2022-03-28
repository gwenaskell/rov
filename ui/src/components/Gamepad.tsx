import { Chip } from "@mui/material";
import Gamepad from "react-gamepad";
import CancelIcon from "@mui/icons-material/Cancel";
import DoneIcon from "@mui/icons-material/Done";
import {
  setAxis,
  setButton,
  setConnected,
  AxisType,
  ButtonType,
} from "../store/gamepadSlice";
import { useDispatch, useSelector } from "../store/hooks";
import WS from "../libs/WebSocket";
import { store } from "../store/store";

// https://dev.to/whoisryosuke/adding-game-controller-input-to-react-5d13

const layout = {
  buttons: [
    "A",
    "B",
    "LB",
    "X",
    "Y",
    "RB",
    "LT",
    "RT",
    "LS",
    "RS",
    "Back",
    "Start",
    "DPadUp",
    "DPadLeft",
    "DPadRight",
    "DPadDown",
  ],
  axis: ["leftY", "-leftX", "rightY", "-rightX", null, null, "padY", "-padX"],
  //buttonAxis: [null, null, null, null, null, null, 'LeftTrigger', 'RightTrigger']
};

function mapAxisName(axis: string): AxisType | '' {
  switch (axis) {
    case "leftX":
    case "leftY":
    case "rightX":
    case "rightY":
    case "padX":
    case "padY":
      return axis;
  }
  return '';
}

function mapButtonName(button: string): ButtonType | '' {
  switch (button) {
    case "LT":
      return "L";
    case "LS":
      return "L2";
    case "RT":
      return "R";
    case "RS":
      return "R2";
    case "A":
    case "B":
    case "X":
    case "Y":
      return button;
    case "DPadLeft":
      return "stickL";
    case "DPadRight":
      return "stickR";
  }
  return '';
}

export default function Controller() {
  const dispatch = useDispatch();

  const connected = useSelector((state) => state.gamepad.connected);

  const buttonChangeHandler = (buttonName: string, down: boolean) => {
    let button = mapButtonName(buttonName);
    if (!button) {
      return
    }

    dispatch(
      setButton({
        axis: button,
        down: down,
      })
    );

    let state = store.getState();

    WS.send(JSON.stringify(state.gamepad));

    console.log("button", buttonName, down);
  };

  const axisChangeHandler = (
    axisName: string,
    value: number,
    previousValue: number
  ) => {
    let axis = mapAxisName(axisName);
    if (axis === '') {
      return;
    }

    value = Math.round(value * 20)*5;

    dispatch(
      setAxis({
        axis: axis,
        value: value,
      })
    );

    let state = store.getState();

    WS.send(JSON.stringify({tm: Date.now(), ...state.gamepad}));

    console.log("axis", axisName, value);
  };

  return (
    <Gamepad
      layout={layout as any}
      onConnect={() => dispatch(setConnected(true))}
      onDisconnect={() => dispatch(setConnected(false))}
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
  );
}
