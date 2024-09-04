import { create } from "zustand";
// import produce from "immer";
import { SceneSlice } from "robot-scene";
import { ProgrammingSlice, DATA_TYPES, SIMPLE_PROPERTY_TYPES } from "open-vp";
import { programSpec } from "./programSpec";
import { subscribeWithSelector } from "zustand/middleware";
import { immer } from "zustand/middleware/immer";
import { DEFAULTS } from "./defaults";
import { shape2item, state2Lines, state2tfs } from "./helpers/InfoParsing";
import { shallow } from "zustand/shallow";
import { invoke } from "@tauri-apps/api/tauri";
import { listen } from "@tauri-apps/api/event";
import { clamp } from "lodash";
import {
  allBehaviorProperties,
  STATE_TYPES,
} from "./Constants";
import _, {
  mapValues,
  pickBy,
  pick,
  uniqWith,
  isEqual,
  last,
  cloneDeep,
  fromPairs,
  findKey
} from "lodash";
import { bp2lik, bp2vis, rs2bp } from "./helpers/Conversion";

const DEFAULT_OBJECTIVE = {
  type: "SmoothnessMacro",
  name: "SmoothnessMacro",
  weight: 10,
};

const store = (set, get) => ({
  pendingTransition: null,
  loaded: true,
  setLoaded: (loaded) => {
    // console.log('new loaded',loaded);
    set({ loaded });
    // console.log('new loaded test',get().loaded);
  },
  showCollision: false,
  setShowCollision: (showCollision) => set({ showCollision }),
  currentState: null,
  stateGoals: {},
  stateWeights: {},
  initiateTransition: (fromNode, toNode) =>
    set((state) => {
      // Should do checking to see whether this is a valid transition,
      // and do any relevant interpolations.
      // For now just immediately transition
      if (state.pendingTransition) {
        clearTimeout(state.pendingTransition);
      }
      console.log(`initiating transition ${fromNode} -> ${toNode}`);
      for (const [key, value] of Object.entries(state.programData)) {
        if (allBehaviorProperties.includes(value.type) && value.selected) {
          state.programData[key].selected = false;
        }
      }

      if (state.programData[fromNode]) {
        state.programData[fromNode].selected = false;
      }
      
      state.programData[toNode].selected = true;

      state.currentState = toNode;

      // Schedule a transition if a timed transition exists
      let transitionTime = Infinity;
      let transitionNode = null;
      Object.values(state.programData)
        .filter(
          (d) =>
            d.dataType === DATA_TYPES.CONNECTION &&
            d.parent.id === toNode &&
            d.mode === SIMPLE_PROPERTY_TYPES.NUMBER
        )
        .forEach((d) => {
          if (Number(d.name) < transitionTime) {
            transitionTime = Number(d.name);
            transitionNode = d.child.id;
          }
        });
      if (transitionNode) {
        const transitionFn = get().initiateTransition;
        state.pendingTransition = setTimeout(
          () => transitionFn(toNode, transitionNode),
          transitionTime * 1000
        );
      }
    }),
  teleport: (data, other) => {
    const currentState = get().currentState;
    console.log(other);
    get().initiateTransition(currentState, data.id);
  },
  isValid: false,
  urdf: DEFAULTS.urdf,
  setUrdf: (urdf) => set({ urdf }),
  rootBounds: [
    { value: 0.0, delta: 0.0 },
    { value: 0.0, delta: 0.0 },
    { value: 0.0, delta: 0.0 }, // Translational
    { value: 0.0, delta: 0.0 },
    { value: 0.0, delta: 0.0 },
    { value: 0.0, delta: 0.0 }, // Rotational
  ],
  setRootBound: (value, idx, isValue) =>
    set((state) => {
      state.rootBounds[idx][isValue ? "value" : "delta"] = Number(value);
    }),
  persistentShapes: [],
  objectives: {},
  goals: {},
  weights: {},
  links: [],
  joints: [],
  ...SceneSlice(set, get), // default robot-scene slice
  robotMeshes: {},
  feedbackMeshes: {},
  proximityLines: {},
  texts: {
    onload: {
      value: "Configure URDF to Begin",
      frame: "world",
      position: { x: 0, y: 0, z: (time) => 0.25 * Math.sin(time / 2000) + 0.5 },
      color: { r: 255, g: 255, b: 255, a: 1 },
    },
  },
  ...ProgrammingSlice(set, get), // default programming slice for simple-vp,
  tabs: [
    {
      title:'Main',
      id: 'default',
      visible: true,
      blocks: ["powerOnType-2c880f27-1777-48b8-852e-861cc5c2ed0a"]
    },
    {
      title:'Scratch',
      id: 'scratch',
      visible: true,
      blocks: []
    }
  ],
  activeTab:'default',
  programSpec,
  setTfs: (robotstate) =>
    set((state) => {
      const tfs = state2tfs(robotstate);
      const proximityLines = state.showCollision ? state2Lines(robotstate) : {};
      // console.log("setting tfs")
      state.tfs = tfs;
      state.proximityLines = proximityLines;
    }),
  setJointScalar: (bpID, value) => set((state) => {

    if (state.programData[bpID]) {
      state.programData[bpID].properties.scalar = value;



    }
  }),
  setBlockSelection: (id, id2) =>
    set((state) => {

      if (state.programData[id] && state.programData[id2]) {
        state.programData[id].selected = true;
        state.programData[id2].selected = false;

      }
    }),
  setDefault: () =>
    set({
      loaded: true,
      programSpec,
      currentState: "powerOnType-2c880f27-1777-48b8-852e-861cc5c2ed0a",
      programData: {
        "powerOnType-2c880f27-1777-48b8-852e-861cc5c2ed0a": {
          argumentBlockData: [],
          canDelete: false,
          canEdit: true,
          dataType: "INSTANCE",
          editing: undefined,
          id: "powerOnType-2c880f27-1777-48b8-852e-861cc5c2ed0a",
          name: "PowerOn",
          position: { x: 100, y: 0 },
          properties: {},
          refData: null,
          selected: true,
          type: "powerOnType",
        },
      },
    }),
  updateProgramSpec: (links, joints) =>
    set((state) => {
      console.log("updating programSpec");
      const objectTypes = programSpec.objectTypes;
      let jointOptions = joints
        .filter((j) => j.jointType !== "fixed" && !j.mimic)
        .map((j) => ({
          label: j.name,
          value: j.name,
        }))
        .sort((a, b) => {
          if (a.label.toLowerCase() < b.label.toLowerCase()) {
            return -1;
          } else {
            return 1;
          }
        });
      let linkOptions = links
        .map((l) => ({ label: l.name, value: l.name }))
        .sort((a, b) => {
          if (a.label.toLowerCase() < b.label.toLowerCase()) {
            return -1;
          } else {
            return 1;
          }
        });
      // linkOptions.sort((a,b)=>{if (a.name.toLowerCase() < b.name.toLowerCase()) {return -1} else {return 1}});
      // console.log(objectTypes)
      state.programSpec.objectTypes = mapValues(
        objectTypes,
        (objectType, key) => {
          let newObjectType = cloneDeep(objectType);
          if (allBehaviorProperties.includes(key)) {
            ["link", "link1", "link2"].forEach((prop) => {
              if (newObjectType?.properties[prop] !== undefined) {
                newObjectType.properties[prop].options = linkOptions;
                newObjectType.properties[prop].default =
                  last(linkOptions).value || "";
              }
            });
            ["joint", "joint1", "joint2"].forEach((prop) => {
              if (newObjectType?.properties[prop] !== undefined) {
                newObjectType.properties[prop].options = jointOptions;
                newObjectType.properties[prop].default =
                  last(jointOptions).value || "";
              }
            });
          }
          // console.log('new',newObjectType)
          return newObjectType;
        }
      );
      // set({loaded:true,programSpec:{drawers:programSpec.drawers,objectTypes:newObjectTypes}})
    }),
  onMove: (id, source, worldTransform, localTransform) =>
    set((state) => {
      let bpId = id;
      let shapeFlag = null;
      ["-top", "-bottom"].forEach((flag) => {
        if (id.includes(flag)) {
          id = id.replace(flag, "");
          shapeFlag = flag.replace("-", "");
        }
      });

      const newBp = rs2bp({
        current: state.programData[bpId],
        worldTransform,
        localTransform,
        source,
        flag: shapeFlag,
        joints: get().joints
      });

      if (newBp) {
        state.programData[bpId] = newBp;
      }
    }),
});

const immerStore = immer(store);

const useStore = create(subscribeWithSelector(immerStore));

const setTfs = useStore.getState().setTfs;
const setBlockSelection = useStore.getState().setBlockSelection;
const setJointScalar = useStore.getState().setJointScalar;

listen("solution-calculated", (event) => {
  if (event.payload) {
    setTfs(event.payload);
  }
});

// Update feedback/input meshes when goals/state change
useStore.subscribe(
  (state) => {
    let bps = [];
    state.programData[state.currentState]?.properties?.children?.forEach(child => {
      if (state.programData[child].type === 'groupType') {
        state.programData[child]?.properties?.children?.forEach(childChild => {
          bps.push(state.programData[childChild])
        })
      } else {
        bps.push(state.programData[child])
      }
    });
    return bps
  },
  (activeBehaviorProperties) => {
    const joints = useStore.getState().joints;
    // Create feedback meshes
    let feedbackMeshes = {};
    let lines = {};
    let hulls = {};
    activeBehaviorProperties.forEach((bp) => {
      const goalFeedbackItems = bp2vis(bp, joints);
      goalFeedbackItems.forEach(({ group, id, data }) => {
        if (group === "items") {
          feedbackMeshes[id] = data;
        } else if (group === "lines") {
          lines[id] = data;
        } else if (group === "hulls") {
          hulls[id] = data;
        }
      });
    });
    console.log("feedbackMeshes", feedbackMeshes);
    useStore.setState({ feedbackMeshes, lines, hulls });
    // console.log('',state.feedbackMeshes)
  },
  { equalityFn: shallow }
);

//Making new subscriber to track unique objectives-------------------------------------------------
useStore.subscribe(
  (state) =>
    mapValues(state.programData, (v) => ({
      type: v.type,
      properties: v.properties,
      id: v.id,
    })),
  (programData) => {
    // let behaviorProperties = [];
    // let states = [];

    const behaviorProperties = pickBy(programData, (d) =>
      allBehaviorProperties.includes(d.type)
    );
    const states = pickBy(programData, (d) => STATE_TYPES.includes(d.type));
    const likValues = {
      default: { objective: DEFAULT_OBJECTIVE, goal: null },
      ...mapValues(behaviorProperties, bp2lik),
    };
    // Creates a new object with only unique values;
    const objectives = mapValues(
      fromPairs(
        uniqWith(Object.entries(likValues), ([_k1, v1], [_k2, v2]) =>
          isEqual(v1.objective, v2.objective)
        )
      ),
      (v) => v.objective
    );

    // Generate a lookup of state goals that can be sent to livelytk
    const stateGoals = mapValues(states, (state) => {
      let currentStateGoals = {};
      state.properties?.children?.forEach((key) => {
        if (programData[key]?.type === "groupType") {
          programData[key].properties.children.forEach((groupChildKey) => {
            const childData = bp2lik(programData[groupChildKey]);
            if (!childData.goal) return;
            let matchedObjKey = findKey(objectives, (objective) =>
              isEqual(objective, childData.objective)
            );
            if (matchedObjKey) {
              currentStateGoals[matchedObjKey] = childData.goal;
            }
          });
        } else {
          const childData = bp2lik(programData[key]);
          if (!childData.goal) return;
          let matchedObjKey = findKey(objectives, (objective) =>
            isEqual(objective, childData.objective)
          );
          if (matchedObjKey) {
            currentStateGoals[matchedObjKey] = childData.goal;
          }
        }
      });
      return currentStateGoals;
    });

    // Generate a lookup of state weights that can be sent to livelytk
    const stateWeights = mapValues(states, (state) => {
      let currentStateWeights = {};
      state.properties?.children?.forEach((key, idx) => {
        if (programData[key]?.type === "groupType") {
          programData[key].properties.children.forEach((groupChildKey) => {
            const childData = bp2lik(programData[groupChildKey]);
            let matchedObjKey = findKey(objectives, (objective) =>
              isEqual(objective, childData.objective)
            );
            if (matchedObjKey) {
              currentStateWeights[matchedObjKey] =
                programData[key].properties.priority * 50 / Math.pow(Math.E, idx / state.properties.children.length);
            } else {
              return 0;
            }
          });
        } else {
          const childData = bp2lik(programData[key]);
          let matchedObjKey = findKey(objectives, (objective) =>
            isEqual(objective, childData.objective)
          );
          if (matchedObjKey) {
            currentStateWeights[matchedObjKey] =
              50 / Math.pow(Math.E, idx / state.properties.children.length);
            if (programData[key].properties.prioritize) {
              currentStateWeights[matchedObjKey] *= 10;
            }
          } else {
            return 0;
          }
        }
      });
      return currentStateWeights;
    });

    console.log("stateInfo", { objectives, stateGoals, stateWeights });

    useStore.setState({
      objectives,
      stateGoals,
      stateWeights,
    });
    // console.log('uniqueObjectives',{uniqueObjectives,stateGoals,stateWeights})
  },
  { equalityFn: isEqual }
);

// Handle updating the urdf
useStore.subscribe(
  (state) => ({ urdf: state.urdf, rootBounds: state.rootBounds }),
  async ({ urdf, rootBounds }) => {
    const result = await invoke("update_urdf", {
      urdf,
      rootBounds: rootBounds.map((b) => [b.value, b.delta]),
    });
    // console.log(rootBounds.map(b=>(b.value,b.delta)))
    // await invoke("update_root_bounds", {
    //   rootBounds: rootBounds.map((b) => [b.value, b.delta]),
    // });
    // const worker = useStore.getState().solverWorker;
    if (result) {
      const initialState = await invoke("solve");
      setTfs(initialState);
      useStore.setState({
        links: result.links,
        joints: result.joints,
        isValid: true,
      });
    } else {
      useStore.setState({ links: [], joints: [], isValid: false });
    }
  },
  { equalityFn: shallow }
);

// Handle updating robot meshes when links change
useStore.subscribe(
  (state) => ({ links: state.links, showCollision: state.showCollision }),
  ({ links, showCollision }) => {
    let robotMeshes = {};
    links.forEach((link) => {
      link.visuals.forEach((visual, i) => {
        robotMeshes[`visual-${link.name}-${i}`] = shape2item(visual, false);
      });
      if (showCollision) {
        link.collisions.forEach((collision, i) => {
          robotMeshes[`collision-${link.name}-${i}`] = shape2item(
            collision,
            true
          );
        });
      }
    });
    useStore.setState({ robotMeshes });
  },
  { equalityFn: shallow }
);

// Update the program spec when links/joints change
useStore.subscribe(
  (state) => ({ links: state.links, joints: state.joints }),
  ({ links, joints }) => useStore.getState().updateProgramSpec(links, joints),
  { equalityFn: shallow }
);
// Applying joint scale to each joints when switching 
useStore.subscribe((state) => ({
  states: state.programData,
  joints: state.joints
}), (states, joints) => {

  for (const [key, item] of Object.entries(states.states)) {

    if (allBehaviorProperties.includes(item.type)) {
      if (item.properties.joint) {


        for (let i = 0; i < joints.joints.length; i++) {

          if (joints.joints[i].name === item.properties.joint) {

            const upperBound = joints.joints[i].upperBound;
            const lowerBound = joints.joints[i].lowerBound;
            const value = clamp(item.properties.scalar, lowerBound, upperBound);
            console.log(item.id, value);
            setJointScalar(item.id, value);
            break;
          }
        }
      }
    }

    //item.properties
  }


  // console.log("programData : " , states);
  // console.log("joints : ", joints);

}, { equalityFn: shallow });
// Limit only one behavior property being selected
useStore.subscribe(

  (state) => mapValues(
    pickBy(state.programData, (d) => allBehaviorProperties.includes(d.type)),
    d => d.selected
  ),
  (newSelected, pastSelected) => {


    if (!(_.isEqual(newSelected, pastSelected))) {

      if (Object.keys(newSelected).length === Object.keys(pastSelected).length) {

        let setFalseID = "";
        let setTrueID = "";
        for (const [key, selected] of Object.entries(newSelected)) {

          if (!pastSelected[key] && selected === true) {

            setTrueID = key;

          }
          if (pastSelected[key] === true && selected === true) {

            setFalseID = key;

          }



        }


        if (setFalseID !== "" && setTrueID !== "") {
          setBlockSelection(setTrueID, setFalseID);
        }

      }



    }


    // If one is selected in past and current, and another was not selected in past and is now selected,
    // then set the one selected both times to false, and keep the one that is now selected.
    //
    // setBlockSelection(pastId,false)
  },
  { equalityFn: shallow }
);

// Merge feedback/robot meshes when either updates
useStore.subscribe(
  (state) => ({
    robotMeshes: state.robotMeshes,
    feedbackMeshes: state.feedbackMeshes,
    proximityLines: state.proximityLines,
  }),
  ({ robotMeshes, feedbackMeshes, proximityLines }) => {
    useStore.setState({
      items: {
        ...robotMeshes,
        ...feedbackMeshes,
        // collision: {
        //   shape: "cube",
        //   frame: "world",
        //   name: "Table",
        //   scale: { x: 0.5, y: 0.7, z: 0.2 },
        //   position: { x: 0.5, y: 0, z: 0 },
        //   rotation: { w: 1, x: 0, y: 0, z: 0 },
        //   color: { r: 255, g: 0, b: 0, a: 1 },
        //   wireframe: true,
        // },
      },
      texts: {},
      lines: proximityLines,
    });
  },
  { equalityFn: shallow }
);

// When the goals/weights change, assign them to the new overarching target
useStore.subscribe(
  (state) => ({
    // currentState: state.currentState,
    goals: state.stateGoals[state.currentState],
    weights: state.stateWeights[state.currentState],
    // objectives: state.objectives,
  }),
  (newValues) => {
    console.log('new goals/weights',newValues.goals)
    if (newValues.goals && newValues.weights) {
      useStore.setState({
        goals: newValues.goals, //will say nextGoals  CHANGED
        weights: newValues.weights, //will say nextWeights  CHANGED
        // goals:newValues.goals, weights:newValues.weights // We will remove this when the function below is finished
      });
    }
  },
  { equalityFn: shallow }
);

// Subsriber to update solving ------------------------------------------------------------------------------
// useStore.subscribe(
//   (state) => ({
//     goals: state.goals,
//     weights: state.weights,
//     nextGoals: state.nextGoals,
//     nextWeights: state.nextWeights
//   }),
//   (newValues) => {
//     if (newValues.nextGoals && newValues.nextWeights) {
//       //console.log('new goals/weights to interp',{newValues});
//       //console.log("Goals: ", newValues.goals)
//       //console.log("Weights: ", newValues.weights)
//       //console.log("nextGoals: ", newValues.nextGoals)
//       //console.log("nextWeights: ", newValues.nextWeights)

//       //Check if goals are in nextGoals (if so, interpolate between them)
//       for (let i in newValues.goals){
//         for (let j in newValues.nextGoals){
//           //Check if element in goals is in nextGoals
//           if (newValues.goals[i] == newValues.nextGoals[j]){
//             let tempInterp;
//             let tempOldGoalValue;
//             let tempNewGoalValue;
//             let stepSize = 0.5;
//             let oldQuat = 0;//Convert tempOld to quaternion
//             let newQuat = 0;
//             //console.log("DAK PRINT: ", newValues.goals[i])
//             if (Object.keys(newValues.goals[i])[0] == "Translation"){
//               //Translation- Use converstion I spoke with Andy about (make a 1 vector)
//               tempOldGoalValue = Object.values(newValues.goals[i])[0]
//               tempNewGoalValue = Object.values(newValues.goals[j])[0]
//               //Interpolate
//               tempInterp = tempOldGoalValue.add(tempNewGoalValue.sub(tempOldGoalValue) * stepSize)

//             }
//             else if (Object.keys(newValues.goals[i])[0] == "Rotation"){
//               //Rotation- 3 vector (convert to quaternion and use rotateTowards)
//               tempOldGoalValue = Object.values(newValues.goals[i])[0]
//               tempNewGoalValue = Object.values(newValues.goals[j])[0]
//               oldQuat = 0; //Convert tempOld to quaternion
//               newQuat = 0; //Convert tempNew to quaternion
//               //Interpolate
//               tempInterp = oldQuat.rotateTowards(newQuat, stepSize)

//             }
//             else if (Object.keys(newValues.goals[i])[0] == "Size"){
//               //Size- 3 vector (three linear interpolations)
//               tempOldGoalValue = Object.values(newValues.goals[i])[0]
//               tempNewGoalValue = Object.values(newValues.goals[j])[0]
//               //Interpolate
//               tempInterp = tempOldGoalValue.lerp(tempNewGoalValue, stepSize)

//             }
//             else if (Object.keys(newValues.goals[i])[0] == "Ellipse"){
//               //Elipse- translate, rotate, size
//               tempOldGoalValue = Object.values(newValues.goals[i])[0]
//               tempNewGoalValue = Object.values(newValues.goals[j])[0]
//               //Interpolate
//               //tempInterp =

//             }
//             else if (Object.keys(newValues.goals[i])[0] == "ScalarRange"){
//               //Scalar Range- two linearly interpolate
//               tempOldGoalValue = Object.values(newValues.goals[i])[0]
//               tempNewGoalValue = Object.values(newValues.goals[j])[0]

//             }
//           }
//           //Interpolate between weights
//         }
//       }

//       //Check if nextGoals are in goals (if not, add nextGoal values to interpolation values)
//       for (let x in newValues.nextGoal){
//         for (let y in newValues.goals){
//           //Check if element in nextGoals is in goals
//           if (newValues.goals[x] = newValues.nextGoals[y]){
//             break;
//           }
//           //Add nextGoal values and weights to interpolation values
//         }
//       }

//       // Do interpolation here
//       // useStore.setState({ goals: interpolatedGoals, weights: interpolatedWeights });
//        useStore.setState({ goals: newValues.nextGoals, weights: newValues.nextWeights });
//     }
//   },
//   { equalityFn: shallow }
// );
//-------------------------------------------------------------------------------------------------

// Listen for changes to goals, weights, objectives and send them to the backend
useStore.subscribe(
  (state) => ({
    goals: state.goals,
    weights: state.weights,
    objectives: state.objectives,
    isSource: state.programData?.[state.currentState]?.type === "powerOnType",
  }),
  (newValues, pastValues) => {
    console.log("new values to invoke");
    if (newValues.isSource) {
      invoke("reset");
    } else if (
      newValues.goals &&
      newValues.weights &&
      newValues.objectives === pastValues.objectives
    ) {
      console.log("Updating solver goals/weights", {
        goals: newValues.goals,
        weights: newValues.weights,
      });
      // useStore.setState({ goals: newValues.goals, weights: newValues.weights });
      invoke("update_goals_and_weights", {
        goals: newValues.goals,
        weights: newValues.weights,
      });
    } else if (
      newValues.goals &&
      newValues.weights &&
      newValues.objectives !== pastValues.objectives
    ) {
      console.log("Updating solver props", newValues);
      // useStore.setState({ goals: newValues.goals, weights: newValues.weights });
      invoke("update_objectives_and_goals_and_weights", {
        objectives: newValues.objectives,
        goals: newValues.goals,
        weights: newValues.weights,
      });
    } else if (newValues.stateType !== "stateType") {
      console.log("Ignoring because not stateType");
    }
  },
  { equalityFn: shallow }
);

useStore.subscribe(
  (state) => state.persistentShapes,
  (shapes) => {
    invoke("update_shapes", { shapes });
  },
  { equalityFn: shallow }
);

// Log current values
useStore.subscribe(
  (state) => pick(state, ["currentState", "goals", "weights", "objectives"]),
  (c, _) => {
    console.log(pick(c, ["currentState", "goals", "weights", "objectives"]));
  },
  { equalityFn: shallow }
);

// Finally, set the program based on the spec and solver instance
// const solverWorker = new SolverWorker();
// const solverWorkerInstance = Comlink.wrap(solverWorker);
// useStore.setState({ programSpec, solverWorker: solverWorkerInstance });
// useStore.setState({programData: {'s':instanceTemplateFromSpec('stateType',programSpec.objectTypes.stateType,false)}})
useStore.setState({
  loaded: true,
  programSpec,
  currentState: "powerOnType-2c880f27-1777-48b8-852e-861cc5c2ed0a",
  persistentShapes: [
    // {
    //   type: "Box",
    //   name: "Table",
    //   frame: "world",
    //   physical: true,
    //   x: 0.5,
    //   y: 0.7,
    //   z: 0.2,
    //   localTransform: { translation: [0.5, 0, 0], rotation: [0, 0, 0, 1] },
    // },
  ],
  programData: {
    "powerOnType-2c880f27-1777-48b8-852e-861cc5c2ed0a": {
      argumentBlockData: [],
      canDelete: false,
      canEdit: true,
      dataType: "INSTANCE",
      editing: undefined,
      id: "powerOnType-2c880f27-1777-48b8-852e-861cc5c2ed0a",
      name: "Power On",
      position: { x: 100, y: 0 },
      properties: {},
      refData: null,
      selected: true,
      type: "powerOnType",
    },
    "powerOffType-sefess-sesff-r454-0f9f-1089dsisdis": {
      argumentBlockData: [],
      canDelete: false,
      canEdit: true,
      dataType: "INSTANCE",
      editing: undefined,
      id: "powerOffType-sefess-sesff-r454-0f9f-1089dsisdis",
      name: "Power Off",
      position: { x: 100, y: 200 },
      properties: {},
      refData: null,
      selected: false,
      type: "powerOffType",
    },
  },
});

// console.log('built store')

export default useStore;

// Object.keys(programSpec.objectTypes).filter(o=>allBehaviorProperties.includes(o)).forEach(bp=>console.log(bp,bp2lik(instanceTemplateFromSpec(bp,programSpec.objectTypes[bp],false))))
