import * as React from 'react';
import { Grid, IconButton } from '@mui/material';
import { VolumeUp, VolumeOff } from '@mui/icons-material';
import { useDispatch, useSelector } from '../store/hooks';
import { setMute } from '../store/controlsSlice'

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
      <IconButton onClick={() => dispatch(setMute(!muted))}>
        {muted ? <VolumeOff fontSize='large'/> : <VolumeUp fontSize='large'/>}
      </IconButton>
    </Grid>
  );
};


export default RightControls;