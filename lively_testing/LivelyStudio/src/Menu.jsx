import React from "react";
import { Alert, IconButton, Badge, AppBar, Toolbar, Typography } from "@mui/material";
import { DropMenu, RootSettings, TransitionDropdown } from "./Elements/DropMenu";
import useStore from "./store";
import { FiThumbsUp, FiSettings, FiGrid, FiEye, FiCodepen } from "react-icons/fi";
import { DEFAULTS } from "./defaults";
import DefaultRobots from "./DefaultRobots";
import { shallow } from "zustand/shallow";
// import { StatusInfo, StatusGood, StatusWarning, FormDown } from 'grommet-icons';

export const Menu = ({ mode, setMode }) => {
  const urdf = useStore((state) => state.urdf,shallow);
  const setUrdf = useStore((state) => state.setUrdf,shallow);
  const isValid = useStore((state) => state.isValid,shallow);
  const showCollision = useStore(state=>state.showCollision,shallow);
  const setShowCollision = useStore(state=>state.setShowCollision,shallow);
  const marker = urdf === DEFAULTS.urdf ? "warning" : !isValid ? "error" : null;

  return (
    <AppBar position="static" component='nav' >
      <Toolbar variant="dense">
        {/* <Alert
        sx={{ height: "30px" }}
        variant="filled"
        severity="success"
        icon={<FiThumbsUp />}
      >
        Some alert text
      </Alert> */}
      <Typography variant='h6' component='div' sx={{flexGrow:1}}>
        {mode=== 'setup' ? "Setup" : "Editor"}
      </Typography>
        {mode === "setup" ? (
          <>
            <DropMenu
              size="xsmall"
              label="Defaults"
              onSelect={(v) => setUrdf(DefaultRobots[v])}
              options={[
                { value: "ur3", label: "UR3" },
                { value: "ur3robotiq", label: "UR3+Robotiq" },
                { value: "ur5robotiq", label: "UR5+Robotiq" },
                { value: "panda", label: "Panda" },
                { value: "pepper", label: "Pepper" },
                { value: "nao", label: "Nao" },
              ]}
            />
            <RootSettings/>
          </>
        ) : <TransitionDropdown/>}
        
        <Badge
          invisible={!showCollision}
          color='error'
          variant="dot"
          overlap="circular"
        >
          <IconButton
            size="small"
            color="pop"
            onClick={() => setShowCollision(!showCollision)}
          >
            <FiCodepen/>
          </IconButton>
        </Badge>
        <Badge
          invisible={marker === null}
          color={marker ? marker : "pop"}
          badgeContent={marker === "warning" || "error" ? "!" : null}
          overlap="circular"
        >
          <IconButton
            size="small"
            //disabled={mode === "setup" && marker !== null}
            color="pop"
            onClick={() => setMode(mode === "default" ? "setup" : "default")}
          >
            {mode === "default" ? <FiSettings /> : <FiGrid />}
          </IconButton>
        </Badge>
      </Toolbar>
    </AppBar>
  );
};
