import React, { useEffect } from "react";
import { store } from "./store/store";
import logo from "./logo.svg";
import "./App.css";
import LeftControls from "./components/LeftControls";
import Rightcontrols from "./components/RightControls";
import { Grid } from "@mui/material";
import Header from "./components/Header";
import Video from "./components/Video";
import WS from "./libs/WebSocket";


function App() {
  useEffect(() => {
    WS.open_connection(
      (payload: string) => {
        console.log(payload);
        store.dispatch(()=>{
        });
      }
    );
  });

  return (
    <div className="App">
      <Video />
      <Grid
        className="AppGrid"
        container
        spacing={0}
        justifyContent="space-between"
        alignItems="center"
        width="100%"
        margin="0"
      >
        <Grid item xs={12}>
          <Header />
        </Grid>

        <Grid item xs={1}>
          <LeftControls />
        </Grid>
        <img src={logo} className="App-logo" alt="logo" hidden />
        <Grid item xs={1}>
          <Rightcontrols />
        </Grid>
      </Grid>
    </div>
  );
}

export default App;
