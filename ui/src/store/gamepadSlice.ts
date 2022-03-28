import { createSlice, PayloadAction } from "@reduxjs/toolkit";

export type AxisType = 'leftX' | 'leftY' | 'rightX' | 'rightY' | 'padX' | 'padY'

export type AxisActionType = {
  axis: AxisType
  value: number
}

export type ButtonType = 'A' | 'B' | 'X' | 'Y' | 'L' | 'L2' | 'R' | 'R2'| 'stickL' | 'stickR'

export type ButtonActionType = {
  axis: ButtonType
  down: boolean
}

const initialState = {
  connected: false,
  sticks: {
    leftX: 0,
    leftY: 0,
    rightX: 0,
    rightY: 0,
    padX: 0,
    padY: 0,
  },
  buttons: {
    A: false,
    B: false,
    X: false,
    Y: false,
    L: false,
    L2: false,
    R: false,
    R2: false,
    stickL: false,
    stickR: false,
  }
}

const gamepadSlice = createSlice({
  name: "gamepad",
  initialState: initialState,
  reducers: {
    setAxis: (state, action: PayloadAction<AxisActionType>) => {
      state.sticks[action.payload.axis] = action.payload.value;
    },
    setButton: (state, action: PayloadAction<ButtonActionType>) => {
      state.buttons[action.payload.axis] = action.payload.down;
    },
    setConnected: (state, action: PayloadAction<boolean>) => {
      if (action.payload) {
        state.connected = action.payload;
      } else {
        return initialState // reset
      }
    },
  },
});

export const { setAxis, setButton, setConnected } = gamepadSlice.actions;

export default gamepadSlice.reducer;
