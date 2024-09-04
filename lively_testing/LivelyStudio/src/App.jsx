import { Grommet, Grid, Box, Layer } from "grommet";
import { getTheme } from "./theme";
import { Scene } from "robot-scene";
import { Environment } from "open-vp";
import useStore from "./store";
import useMeasure from "react-use-measure";
import { useState } from "react";
import { URDF } from "./Tabs/URDF";
import { Menu } from "./Menu";
// import Button from "@mui/material/Button";
// import { Icon, IconButton } from "@mui/material";
import { ThemeProvider, createTheme } from "@mui/material/styles";
import { ReflexContainer, ReflexSplitter, ReflexElement } from "react-reflex";
import MeshLookupTable from "./Meshes";
import { DEFAULTS } from "./defaults";
import { Backdrop, CircularProgress } from "@mui/material";
import { shallow } from "zustand/shallow";
import "react-reflex/styles.css";
import 'reactflow/dist/style.css';
import './App.css';

function App() {
  const loaded = useStore((state) => state.loaded, shallow);
  const primaryColor = "#FEDE00";
  const theme = getTheme(primaryColor);
  const [editorRef, editorBounds] = useMeasure();
  const urdf = useStore((state) => state.urdf);
  const [mode, setMode] = useState(
    urdf === DEFAULTS.urdf ? "setup" : "default"
  );

  const muiTheme = createTheme({
    palette: {
      mode: "dark",
      primaryColor: {
        main: primaryColor,
      },
      quiet: {
        main: "#444",
        darker: "#333",
      },
      pop: {
        main: "#999",
        darker: "#777",
      },
    },
  });

  // console.log('rerender')

  return (
    <ThemeProvider theme={muiTheme}>
      <Grommet full theme={theme}>
        <Box width="100vw" height="100vh" direction="row">
          {loaded ? (
            <ReflexContainer orientation="vertical">
              <ReflexElement>
              <Box
                flex
                fill
                background="#111"
                // pad="xxsmall"
                style={{ overflow: "hidden" }}
              >
                <Scene
                  // displayTfs={true}
                  displayGrid={true}
                  isPolar={false}
                  backgroundColor="#1e1e1e"
                  planeColor="#141414"
                  highlightColor={primaryColor}
                  plane={0}
                  fov={50}
                  store={useStore}
                  meshLookup={MeshLookupTable}
                  // onPointerMissed={clearFocus}
                  // paused={paused}
                />
              </Box>
              </ReflexElement>
              <ReflexSplitter />
              <ReflexElement>
              <Box flex fill background="#111" direction="column">
                {/* <Box align="center" pad="small" justify="between" direction="row" style={{boxShadow:'2px 2px 1px 1px #11111155'}}> */}
                <Menu mode={mode} setMode={setMode} />
                {/* </Box> */}
                {mode === "default" ? (
                  <Box
                    ref={editorRef}
                    align="center"
                    justify="center"
                    animation="fadeIn"
                    height="100%"
                  >
                    <Environment
                      store={useStore}
                      highlightColor={primaryColor}
                      height={editorBounds.height - 5}
                      width={editorBounds.width - 5}
                      snapToGrid={false}
                    />
                  </Box>
                ) : (
                  <URDF />
                )}
              </Box>
              </ReflexElement>
            </ReflexContainer>
          ) : (
            <Backdrop
              sx={{ color: "#fff", zIndex: (theme) => theme.zIndex.drawer + 1 }}
              open
            >
              <CircularProgress color="inherit" />
            </Backdrop>
          )}
        </Box>
      </Grommet>
    </ThemeProvider>
  );
}

export default App;
