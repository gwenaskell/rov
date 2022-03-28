import Controller from './Gamepad';
import { Grid } from '@mui/material';

export default function Header() {
  return (
    <Grid
      container
      direction="row"
      className="Header"
      alignItems="center"
      justifyContent="flex-end"
      columns={6}
      height="50px"
      paddingRight="5px"
    >
      <Grid xs={1}
      item
      container
      justifyContent="flex-end"
      >
        <Controller/>
      </Grid>
    </Grid>
  );
}