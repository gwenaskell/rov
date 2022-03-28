import { configureStore } from '@reduxjs/toolkit'
import { AnyAction } from 'redux';
import controlsReducer from './controlsSlice'
import gamepadReducer from './gamepadSlice'


export const store = configureStore({
  reducer: {
    controls: controlsReducer,
    gamepad: gamepadReducer,
  }
})

// Infer the `RootState` and `AppDispatch` types from the store itself
export type RootState = ReturnType<typeof store.getState>;
// Inferred type: {posts: PostsState, comments: CommentsState, users: UsersState}
export type AppDispatch = typeof store.dispatch;