import * as React from "react";
import { styled } from "@mui/material/styles";
import Box from "@mui/material/Box";
import Typography from "@mui/material/Typography";
import Slider from "@mui/material/Slider";

// const Separator = styled("div")(
//   ({ theme }) => `
//   height: ${theme.spacing(3)};
// `
// );

interface State {
  value: number;
  connected: boolean;
}

const PrettoSlider = styled(Slider)({
  color: "#013aad",
  width: 2,
  "& .MuiSlider-valueLabel": {
    lineHeight: 1.2,
    fontSize: 12,
    background: "unset",
    padding: 0,
    width: 32,
    height: 32,
    borderRadius: "50% 50% 50% 0",
    backgroundColor: "#024a9c",
    // transform: 'translate(50%, -100%) rotate(-45deg) scale(0)',
    "&:before": { display: "none" },
    "&.MuiSlider-valueLabelOpen": {
      transform: "translate(60%, -15%) rotate(45deg) scale(1)",
    },
    "& > *": {
      transform: "rotate(-45deg)",
    },
  },
});
export default class DepthSlider extends React.Component<any, State> {
  constructor(props: any) {
    super(props);
    this.state = {
      value: 0,
      connected: false,
    };
  }

  setConnected = (connected: boolean) => {
    this.setState({...this.state, connected: connected})
  }

  setDepth = (n: number) => {
    this.setState({...this.state, value: n})
  }

  valuetext = (value: number) => {
    if (!this.state.connected) {
      return "...";
    }
    return `${value} m`;
  };

  render() {
    return (
      <Box sx={{ height: '20vh' }}>
        <PrettoSlider
          track={false}
          aria-labelledby="depth-slider"
          aria-label="Depth slider"
          getAriaValueText={this.valuetext}
          valueLabelFormat={this.valuetext}
          orientation="vertical"
          size="small"
          min={-30}
          defaultValue={0}
          valueLabelDisplay="on"
          max={0}
          disabled
        />
        <Typography id="depth-slider" gutterBottom>
          Depth
        </Typography>
      </Box>
    );
  }
}
