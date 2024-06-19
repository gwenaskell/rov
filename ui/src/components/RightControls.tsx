import * as React from 'react';
import { Grid, IconButton } from '@mui/material';
import { VolumeUp, VolumeOff } from '@mui/icons-material';
import { useDispatch, useSelector } from '../store/hooks';
import { setMute } from '../store/controlsSlice'
import EnginesState from './controls/EnginesState'

const RightControls = () => {
  const dispatch = useDispatch()

  const muted = useSelector((state) => state.controls.mute)

  return (
    <Grid
      container
      direction="column"
      className="Controls"
      alignItems="center"
      color="white"
      paddingTop="20px"
    >
      <Grid
        item
        direction="row"
        xs={1}
      >
        <IconButton onClick={() => dispatch(setMute(!muted))}>
          {muted ? <VolumeOff fontSize='large'/> : <VolumeUp fontSize='large'/>}
        </IconButton>
      </Grid>
      <Grid
        item
        direction="row"
        xs={3}
      >
      <EnginesState/>
      </Grid>
    </Grid>
  );
};


export default RightControls;