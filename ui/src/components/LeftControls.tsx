import Depth from "./Depth";
import { Grid, IconButton } from '@mui/material';
import { BatteryFull } from "@mui/icons-material";

export default function LeftControls() {
  return (
    <Grid
      container
      direction="column"
      className="Controls"
      alignItems="center"
      color="white"
      paddingTop="20px"
      justifyContent="space-between"
      columns={12}
    >
      <Grid item xs={1}>
        <BatteryFull fontSize="large" />   
      </Grid>
      <Grid item xs={3}>
        <Depth />
      </Grid>
      <Grid item xs={2}>
        
      </Grid>
      <Grid item xs={2}>
        
      </Grid>
    </Grid>
  );
}
