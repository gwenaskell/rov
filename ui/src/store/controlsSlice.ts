import { createSlice, PayloadAction } from "@reduxjs/toolkit"

const controlsSlice = createSlice({
  name: "controls",
  initialState: {
    mute: true,
    enginesState: "off",
  },
  reducers: {
    setMute: (state, action: PayloadAction<boolean>) => {
      // Redux Toolkit allows us to write "mutating" logic in reducers. It
      // doesn't actually mutate the state because it uses the immer library,
      // which detects changes to a "draft state" and produces a brand new
      // immutable state based off those changes
      state.mute = action.payload
    },
    setEngineState: (state, action: PayloadAction<StateEnum>) => {
      // Redux Toolkit allows us to write "mutating" logic in reducers. It
      // doesn't actually mutate the state because it uses the immer library,
      // which detects changes to a "draft state" and produces a brand new
      // immutable state based off those changes
      state.enginesState = action.payload
    },
  },
})

export type StateEnum = "off" | "paused" | "running"

export const { setMute, setEngineState } = controlsSlice.actions

export default controlsSlice.reducer
