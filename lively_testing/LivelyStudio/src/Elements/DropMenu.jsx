import React, { useCallback, useState } from "react";
import { Divider, TextField, Badge, IconButton, Button, Menu, MenuItem } from "@mui/material";
import useStore from "../store";
import { shallow } from "zustand/shallow";
import { pickBy } from "lodash";
import { DATA_TYPES } from "open-vp";
import { IoArrowRedo } from "react-icons/io5";
import { styled } from '@mui/material/styles';

const EdgeBadge = styled(Badge)(({ theme }) => ({
  '& .MuiBadge-badge': {
    right: 2,
    top: 5,
    border: `2px solid ${theme.palette.background.paper}`,
    padding: '0 4px',
  },
}));


export const DropMenu = ({ label, options, onSelect }) => {
  const [anchorEl, setAnchorEl] = useState(null);
  const open = Boolean(anchorEl);
  const handleClick = (event) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  return (
    <div>
      <Button
        color="pop"
        id="basic-button"
        aria-controls={open ? "basic-menu" : undefined}
        aria-haspopup="true"
        aria-expanded={open ? "true" : undefined}
        onClick={handleClick}
      >
        {label}
      </Button>
      <Menu
        id="basic-menu"
        anchorEl={anchorEl}
        open={open}
        onClose={handleClose}
        MenuListProps={{
          "aria-labelledby": "basic-button",
        }}
      >
        {options.map((o) => (
          <MenuItem
            key={o.value}
            onClick={() => {
              handleClose();
              if (onSelect) {
                onSelect(o.value);
              }
            }}
          >
            {o.label}
          </MenuItem>
        ))}
      </Menu>
    </div>
  );
};

export const RootSettings = () => {
  const [anchorEl, setAnchorEl] = useState(null);
  const open = Boolean(anchorEl);
  const handleClick = (event) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  const setRootBound = useStore((state) => state.setRootBound, shallow);
  const rootBounds = useStore((state) => state.rootBounds, shallow);

  return (
    <div>
      <Button
        color="pop"
        id="drop-settings-button"
        aria-controls={open ? "drop-settings-menu" : undefined}
        aria-haspopup="true"
        aria-expanded={open ? "true" : undefined}
        onClick={handleClick}
      >
        Root
      </Button>
      <Menu
        id="drop-settings-menu"
        anchorEl={anchorEl}
        open={open}
        onClose={handleClose}
        MenuListProps={{
          "aria-labelledby": "drop-settings-button",
        }}
      >
        <MenuItem sx={{ paddingTop: 1.5 }} key="pos-x">
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Position X"
            value={rootBounds[0].value}
            sx={{ marginRight: 1 }}
            onChange={(e) => setRootBound(e.target.value, 0, true)}
          />
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Position X (Delta)"
            value={rootBounds[0].delta}
            onChange={(e) => setRootBound(e.target.value, 0, false)}
          />
        </MenuItem>
        <MenuItem sx={{ paddingTop: 1.5 }} key="pos-y">
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Position Y"
            value={rootBounds[1].value}
            sx={{ marginRight: 1 }}
            onChange={(e) => setRootBound(e.target.value, 1, true)}
          />
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Position Y (Delta)"
            value={rootBounds[1].delta}
            onChange={(e) => setRootBound(e.target.value, 1, false)}
          />
        </MenuItem>
        <MenuItem sx={{ paddingTop: 1.5 }} key="pos-z">
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Position Z"
            value={rootBounds[2].value}
            sx={{ marginRight: 1 }}
            onChange={(e) => setRootBound(e.target.value, 2, true)}
          />
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Position Z (Delta)"
            value={rootBounds[2].delta}
            onChange={(e) => setRootBound(e.target.value, 2, false)}
          />
        </MenuItem>
        <Divider orientation="horizontal" />
        <MenuItem sx={{ paddingTop: 1.5 }} key="rot-r">
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Rotation R"
            value={rootBounds[3].value}
            sx={{ marginRight: 1 }}
            onChange={(e) => setRootBound(e.target.value, 3, true)}
          />
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Rotation R (Delta)"
            value={rootBounds[3].delta}
            onChange={(e) => setRootBound(e.target.value, 3, false)}
          />
        </MenuItem>
        <MenuItem sx={{ paddingTop: 1.5 }} key="rot-p">
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Rotation P"
            value={rootBounds[4].value}
            sx={{ marginRight: 1 }}
            onChange={(e) => setRootBound(e.target.value, 4, true)}
          />
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Rotation P (Delta)"
            value={rootBounds[4].delta}
            onChange={(e) => setRootBound(e.target.value, 4, false)}
          />
        </MenuItem>
        <MenuItem sx={{ paddingTop: 1.5 }} key="rot-y">
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Rotation Y"
            value={rootBounds[5].value}
            sx={{ marginRight: 1 }}
            onChange={(e) => setRootBound(e.target.value, 5, true)}
          />
          <TextField
            size="small"
            type='number'
            inputProps={{step:0.1}}
            label="Rotation Y (Delta)"
            value={rootBounds[5].delta}
            onChange={(e) => setRootBound(e.target.value, 5, false)}
          />
        </MenuItem>
      </Menu>
    </div>
  );
};

export const TransitionDropdown = () => {
  const [anchorEl, setAnchorEl] = useState(null);
  const open = Boolean(anchorEl);
  const handleClick = (event) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  const initiateTransition = useStore(
    (state) => state.initiateTransition,
    shallow
  );

  const outputs = useStore(
    (state) =>
      pickBy(
        state.programData,
        (d) =>
          d.dataType === DATA_TYPES.CONNECTION &&
          d.parent.id === state.currentState
      ),
    shallow
  );

  const targets = useStore(
    useCallback(
      (state) => {
        let targets = {};
        Object.values(outputs).forEach((output) => {
          // console.log(output.child.id);
          targets[output.child.id] = state.programData[output.child.id];
        });
        return targets;
      },
      [outputs]
    ),
    shallow
  );

  const hasOutputs = Object.keys(outputs).length > 0;
  // console.log({ outputs, targets });

  return (
    <div>
      <EdgeBadge
          // size='small'
          color={hasOutputs ? 'success' : "pop"}
          badgeContent={Object.keys(outputs).length}
          // overlap="circular"
        >
      <IconButton
        color="pop"
        size="small"
        //disabled={mode === "setup" && marker !== null}
        id="drop-transition-button"
        aria-controls={open ? "drop-transition-menu" : undefined}
        aria-haspopup="true"
        aria-expanded={open ? "true" : undefined}
        onClick={handleClick}
      >
        <IoArrowRedo />
      </IconButton></EdgeBadge>
      <Menu
        id="drop-transition-menu"
        anchorEl={anchorEl}
        open={open}
        onClose={handleClose}
        MenuListProps={{
          "aria-labelledby": "drop-transition-button",
        }}
      >
        <Divider orientation="horizontal" textAlign="left">
          Go...
        </Divider>
        {hasOutputs ? (
          Object.entries(outputs).map(([key, edge]) => (
            <MenuItem
              key={key}
              onClick={() => {
                if (edge.mode === "STRING") {
                  initiateTransition(edge.parent.id, edge.child.id);
                  handleClose();
                }
              }}
            >
              {edge.mode === "STRING" ? (
                <>
                  <span
                    style={{
                      marginRight: 2,
                      backgroundColor: "#555",
                      borderRadius: 3,
                      paddingLeft: 5,
                      paddingRight: 5,
                    }}
                  >
                    {edge.name}
                  </span>
                  {"to"}
                </>
              ) : (
                <>
                  {"Transitions in"}
                  <span
                    style={{
                      marginLeft: 2,
                      marginRight: 2,
                      backgroundColor: "#555",
                      borderRadius: 3,
                      paddingLeft: 5,
                      paddingRight: 5,
                    }}
                  >
                    {edge.name}s
                  </span>
                  {"to"}
                </>
              )}
              <span
                style={{
                  marginLeft: 2,
                  backgroundColor: "#7dd6ff",
                  borderRadius: 3,
                  paddingLeft: 5,
                  paddingRight: 5,
                }}
              >
                {targets[edge.child.id].name}
              </span>
            </MenuItem>
          ))
        ) : (
          <MenuItem sx={{ paddingTop: 1.5 }} key="no-transition">
            No Transitions from Current Node
          </MenuItem>
        )}
      </Menu>
    </div>
  );
};
