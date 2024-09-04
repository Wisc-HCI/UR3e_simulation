import { DATA_TYPES, CONNECTIONS, EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "open-vp";
import {
  FiBriefcase,
  FiGrid,
  FiBox,
  FiLogOut,
  FiMoreHorizontal,
  FiFeather
} from "react-icons/fi";
import { ContainerIcon } from "./Elements/Icons/Container";
import { BoundingIcon } from "./Elements/Icons/Bounding";
import { BasicIcon } from "./Elements/Icons/Basic";
import { MatchingIcon } from "./Elements/Icons/Matching";
import { MirroringIcon } from "./Elements/Icons/Mirroring";
import { ForcesIcon } from "./Elements/Icons/Forces";
import { LivelinessIcon } from "./Elements/Icons/Liveliness";
import { IoArrowRedo } from "react-icons/io5";
import { merge } from 'lodash';
import { behaviorPropertyColorBase, behaviorPropertyColorBounding, behaviorPropertyColorMatching, behaviorPropertyColorMirroring, behaviorPropertyColorForces, behaviorPropertyColorLiveliness, behaviorPropertyDrawerBase, behaviorPropertyDrawerBounding, behaviorPropertyDrawerMatching, behaviorPropertyDrawerMirroring, behaviorPropertyDrawerForces, behaviorPropertyDrawerLiveliness } from './Constants.js'

const behaviorPropertyTemplate = {
  type: TYPES.OBJECT,
  // referenceBlock: {
  //   onCanvas: false,
  //   // icon: FiGrid,
  //   extras: [
  //     EXTRA_TYPES.LOCKED_INDICATOR,
  //     {
  //       icon: FiMoreHorizontal,
  //       type: EXTRA_TYPES.DROPDOWN,
  //       contents: [
  //         EXTRA_TYPES.DELETE_BUTTON,
  //         EXTRA_TYPES.DEBUG_TOGGLE,
  //         EXTRA_TYPES.NAME_EDIT_TOGGLE,
  //         EXTRA_TYPES.SELECTION_TOGGLE,
  //       ],
  //     },
  //   ],
  // },
  instanceBlock: {
    onCanvas: false,
    // minified: true,
    // color: "#bdb655",
    // icon: FiGrid,
    hideNewPrefix: true,
    extras: [
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.SELECTION_TOGGLE,
          EXTRA_TYPES.NAME_EDIT_TOGGLE,
          EXTRA_TYPES.COLLAPSE_TOGGLE,
          EXTRA_TYPES.DELETE_BUTTON,
        ],
      },
    ],
  },
  properties: {
    // prioritize: {
    //   name: "Prioritize",
    //   type: SIMPLE_PROPERTY_TYPES.BOOLEAN,
    //   default: false,
    // }
  }
}

//Defining Behavioral Properties------------------------------------------------------------------------
const DEFAULT_FREQUENCY = {
  name: "Frequency",
  type: SIMPLE_PROPERTY_TYPES.NUMBER,
  min: 0,
  max: 20,
  step: 0.1,
  units: 'm/s',
  visualScaling: 1,
  visualPrecision: 1,
  default: 1,
}

const DEFAULT_TRANSLATION = {
  name: "Translation",
  type: SIMPLE_PROPERTY_TYPES.VECTOR3,
  default: [0.0, 0.0, 0.0],
  min: [0.0,0.0,0.0],
  step: 0.01
}

const DEFAULT_ROTATION = {
  name: "Rotation",
  type: SIMPLE_PROPERTY_TYPES.VECTOR3,
  default: [0.0, 0.0, 0.0],
  min: [-360.0,-360.0,-360.0],
  min: [360.0,360.0,360.0],
  step: 1
}

const DEFAULT_SIZE = {
  name: "Size",
  type: SIMPLE_PROPERTY_TYPES.VECTOR3,
  default: [0.05, 0.05, 0.05],
  min: [0.0,0.0,0.0],
  step: 0.01
}

const DEFAULT_SCALAR = {
  name: "Value",
  type: SIMPLE_PROPERTY_TYPES.NUMBER,
  default: 0.0,
  min: Number.NEGATIVE_INFINITY,
  max: Number.POSITIVE_INFINITY,
  step: 0.01
}



//Name, weight, link, translation
const positionMatchBehaviorData = {
  name: 'Position Match',
  instanceBlock: {
    icon: MatchingIcon,
    color: behaviorPropertyColorMatching
  },
  properties: {
    link: {
      name: "Link",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    translation: DEFAULT_TRANSLATION
  }
}

//Name, weight, link, rotation
const orientationMatchBehaviorData = {
  name: 'Orientation Match',
  instanceBlock: {
    icon: MatchingIcon,
    color: behaviorPropertyColorMatching
  },
  properties: {
    link: {
      name: "Link",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    rotation: DEFAULT_ROTATION
  }
}

//Name, weight, link, frequency, size
const positionLivelinessBehaviorData = {
  name: 'Position Liveliness',
  instanceBlock: {
    icon: LivelinessIcon,
    color: behaviorPropertyColorLiveliness
  },
  properties: {
    link: {
      name: "Link",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    frequency: DEFAULT_FREQUENCY,
    size: DEFAULT_SIZE
  }
}

//Name, weight, link, frequency, size
const orientationLivelinessBehaviorData = {
  name: 'Orientation Liveliness',
  instanceBlock: {
    icon: LivelinessIcon,
    color: behaviorPropertyColorLiveliness
  },
  properties: {
    link: {
      name: "Link",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    frequency: DEFAULT_FREQUENCY,
    size: DEFAULT_SIZE
  }
}

//Name, weight, link1, link2, translation
const positionMirroringBehaviorData = {
  name: 'Position Mirroring',
  instanceBlock: {
    icon: MirroringIcon,
    color: behaviorPropertyColorMirroring
  },
  properties: {
    link1: {
      name: "Link 1",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    link2: {
      name: "Link 2",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    translation: {...DEFAULT_TRANSLATION,name:'Offset'}
  }
}

//Name, weight, link1, link2, rotation
const orientationMirroringBehaviorData = {
  name: 'Orientation Mirroring',
  instanceBlock: {
    icon: MirroringIcon,
    color: behaviorPropertyColorMirroring
  },
  properties: {
    link1: {
      name: "Link 1",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    link2: {
      name: "Link 2",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    rotation: {...DEFAULT_ROTATION,name:'Offset'}
  }
}

//Name, weight, link, elipse
const positionBoundingBehaviorData = {
  name: 'Position Bounding',
  instanceBlock: {
    icon: BoundingIcon,
    color: behaviorPropertyColorBounding
  },
  properties: {
    link: {
      name: "Link",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    editMode: {
      name: "Edit Mode",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [{value:'translate',label:'Translate'},{value:'rotate',label:'Rotate'},{value:'scale',label:'Scale'}],
      default: 'translate',
    },
    translation: {...DEFAULT_TRANSLATION,name:'Ellipse Translation'},
    rotation: {...DEFAULT_ROTATION,name:'Ellipse Rotation'},
    size: {...DEFAULT_SIZE,name:'Ellipse Size'}
  }
}

//Name, weight, link
const orientationBoundingBehaviorData = {
  name: 'Orientation Bounding',
  instanceBlock: {
    icon: BoundingIcon,
    color: behaviorPropertyColorBounding
  },
  properties: {
    link: {
      name: "Link",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    rotation: {...DEFAULT_ROTATION,name:'Rotation Center'},
    scalar: {...DEFAULT_SCALAR,name:'Rotation Offset'}
  }
}

//Name, weight, joint, scalar
const jointMatchBehaviorData = {
  name: 'Joint Match',
  instanceBlock: {
    icon: MatchingIcon,
    color: behaviorPropertyColorMatching
  },
  properties: {
    joint: {
      name: "Joint",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    scalar: DEFAULT_SCALAR
  }
}

//Name, weight, link, frequency, scalar
const jointLivelinessBehaviorData = {
  name: 'Joint Liveliness',
  instanceBlock: {
    icon: LivelinessIcon,
    color: behaviorPropertyColorLiveliness
  },
  properties: {
    joint: {
      name: "Joint",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    frequency: DEFAULT_FREQUENCY,
    scalar: {...DEFAULT_SCALAR,name:'Amplitude'}
  }
}

//Name, weight, joint1, joint2, scalar
const jointMirroringBehaviorData = {
  name: 'Joint Mirroring',
  instanceBlock: {
    icon: MirroringIcon,
    color: behaviorPropertyColorMirroring
  },
  properties: {
    joint1: {
      name: "Joint 1",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    joint2: {
      name: "Joint 2",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    scalar: {...DEFAULT_SCALAR,name:'Offset'}
  }
}

//Name, weight
const jointLimitsBehaviorData = {
  name: 'Joint Limits',
  instanceBlock: {
    icon: BasicIcon,
    color: behaviorPropertyColorBase
  },
  properties: {
  }
}

//Name, weight, joint, scalar range
const jointBoundingBehaviorData = {
  name: 'Joint Bounding',
  instanceBlock: {
    icon: BoundingIcon,
    color: behaviorPropertyColorBounding
  },
  properties: {
    joint: {
      name: "Joint",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    scalar: DEFAULT_SCALAR,
    delta: {...DEFAULT_SCALAR,name:'Delta',min:0}
  }
}

//Name, weight
const collisionAvoidanceBehaviorData = {
  name: 'Collision Avoidance',
  instanceBlock: {
    icon: BasicIcon,
    color: behaviorPropertyColorBase
  },
  properties: {
  }
}

//Name, weight
const velocityMinimizationBehaviorData = {
  name: 'Velocity Minimization',
  instanceBlock: {
    icon: BasicIcon,
    color: behaviorPropertyColorBase
  },
  properties: {
  }
}

//Name, weight
const accelerationMinimizationBehaviorData = {
  name: 'Acceleration Minimization',
  instanceBlock: {
    icon: BasicIcon,
    color: behaviorPropertyColorBase
  },
  properties: {
  }
}

//Name, weight
const jerkMinimizationBehaviorData = {
  name: 'Jerk Minimization',
  instanceBlock: {
    icon: BasicIcon,
    color: behaviorPropertyColorBase
  },
  properties: {
  }
}

//Name, weight
const originVelocityMinimizationBehaviorData = {
  name: 'Origin Velocity Minimization',
  instanceBlock: {
    icon: BasicIcon,
    color: behaviorPropertyColorBase
  },
  properties: {
  }
}

//Name, weight
const originAccelerationMinimizationBehaviorData = {
  name: 'Origin Acceleration Minimization',
  instanceBlock: {
    icon: BasicIcon,
    color: behaviorPropertyColorBase
  },
  properties: {
  }
}

//Name, weight
const originJerkMinimizationBehaviorData = {
  name: 'Origin Jerk Minimization',
  instanceBlock: {
    icon: BasicIcon,
    color: behaviorPropertyColorBase
  },
  properties: {
  }
}

//Name, weight, link 1, link 2, frequency, scalar
const relativeMotionLivelinessBehaviorData = {
  name: 'Relative Motion Liveliness',
  instanceBlock: {
    icon: LivelinessIcon,
    color: behaviorPropertyColorLiveliness
  },
  properties: {
    link1: {
      name: "Link 1",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    link2: {
      name: "Link 2",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    frequency: DEFAULT_FREQUENCY,
    scalar: {...DEFAULT_SCALAR,name:'Amplitude'}
  }
}


//Name, weight, link
const gravityBehaviorData = {
  name: 'Gravity',
  instanceBlock: {
    icon: ForcesIcon,
    color: behaviorPropertyColorForces
  },
  properties: {
    link: {
      name: "Link",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
  }
}

//Name, weight
const smoothnessMacroBehaviorData = {
  name: 'Smoothness Macro',
  instanceBlock: {
    icon: BasicIcon,
    color: behaviorPropertyColorBase
  },
  properties: {
  }
}

//Name, weight, link1, link2, scalar
const distanceMatchBehaviorData = {
  name: 'Distance Match',
  instanceBlock: {
    icon: MatchingIcon,
    color: behaviorPropertyColorMatching
  },
  properties: {
    link1: {
      name: "Link 1",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    link2: {
      name: "Link 2",
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: [],
      default: '',
    },
    scalar: {...DEFAULT_SCALAR,name:'Distance'}
  }
}
//---------------------------------------------------------------------------------------------------

export const programSpec = {
  drawers: [
    {
      title: "Structures",
      dataType: DATA_TYPES.INSTANCE,
      objectTypes: ["stateType", 'groupType', "powerOnType", "powerOffType"],
      icon: ContainerIcon,
    },
    {
      //Changed "Hat" to "Goals" and added all new behavioral properties.
      title: "Basic Behavior Properties",
      dataType: DATA_TYPES.INSTANCE,
      objectTypes: behaviorPropertyDrawerBase,
      icon: BasicIcon,
    },
    {
      title: "Bounding Behavior Properties",
      dataType: DATA_TYPES.INSTANCE,
      objectTypes: behaviorPropertyDrawerBounding,
      icon: BoundingIcon,
    },
    {
      title: "Matching Behavior Properties",
      dataType: DATA_TYPES.INSTANCE,
      objectTypes: behaviorPropertyDrawerMatching,
      icon: MatchingIcon,
    },
    {
      title: "Mirroring Behavior Properties",
      dataType: DATA_TYPES.INSTANCE,
      objectTypes: behaviorPropertyDrawerMirroring,
      icon: MirroringIcon,
    },
    {
      title: "Force Behavior Properties",
      dataType: DATA_TYPES.INSTANCE,
      objectTypes: behaviorPropertyDrawerForces,
      icon: ForcesIcon,
    },
    {
      title: "Liveliness Behavior Properties",
      dataType: DATA_TYPES.INSTANCE,
      objectTypes: behaviorPropertyDrawerLiveliness,
      icon: LivelinessIcon,
    },
    //Commented out Boots as this will no longer be needed.
    //{
    //  title: "Boots", //Can get rid of boots
    //  dataType: DATA_TYPES.INSTANCE,
    //  objectTypes: ["bootType"],
    //  icon: FiBox,
    //},
  ],
  objectTypes: {

    // Add all new behavior properties-----------------------------------------------------------------
    positionMatchBehaviorProperty: merge(positionMatchBehaviorData, behaviorPropertyTemplate),
    orientationMatchBehaviorProperty: merge(orientationMatchBehaviorData, behaviorPropertyTemplate),
    positionLivelinessBehaviorProperty: merge(positionLivelinessBehaviorData, behaviorPropertyTemplate),
    orientationLivelinessBehaviorProperty: merge(orientationLivelinessBehaviorData, behaviorPropertyTemplate),
    positionMirroringBehaviorProperty: merge(positionMirroringBehaviorData, behaviorPropertyTemplate),
    orientationMirroringBehaviorProperty: merge(orientationMirroringBehaviorData, behaviorPropertyTemplate),
    positionBoundingBehaviorProperty: merge(positionBoundingBehaviorData, behaviorPropertyTemplate),
    orientationBoundingBehaviorProperty: merge(orientationBoundingBehaviorData, behaviorPropertyTemplate),
    jointMatchBehaviorProperty: merge(jointMatchBehaviorData, behaviorPropertyTemplate),
    jointLivelinessBehaviorProperty: merge(jointLivelinessBehaviorData, behaviorPropertyTemplate),
    jointMirroringBehaviorProperty: merge(jointMirroringBehaviorData, behaviorPropertyTemplate),
    jointLimitsBehaviorProperty: merge(jointLimitsBehaviorData, behaviorPropertyTemplate),
    jointBoundingBehaviorProperty: merge(jointBoundingBehaviorData, behaviorPropertyTemplate),
    collisionAvoidanceBehaviorProperty: merge(collisionAvoidanceBehaviorData, behaviorPropertyTemplate),
    velocityMinimizationBehaviorProperty: merge(velocityMinimizationBehaviorData, behaviorPropertyTemplate),
    accelerationMinimizationBehaviorProperty: merge(accelerationMinimizationBehaviorData, behaviorPropertyTemplate),
    jerkMinimizationBehaviorProperty: merge(jerkMinimizationBehaviorData, behaviorPropertyTemplate),
    originVelocityMinimizationBehaviorProperty: merge(originVelocityMinimizationBehaviorData, behaviorPropertyTemplate),
    originAccelerationMinimizationBehaviorProperty: merge(originAccelerationMinimizationBehaviorData, behaviorPropertyTemplate),
    originJerkMinimizationBehaviorProperty: merge(originJerkMinimizationBehaviorData, behaviorPropertyTemplate),
    relativeMotionLivelinessBehaviorProperty: merge(relativeMotionLivelinessBehaviorData, behaviorPropertyTemplate),
    gravityBehaviorProperty: merge(gravityBehaviorData, behaviorPropertyTemplate),
    smoothnessMacroBehaviorProperty: merge(smoothnessMacroBehaviorData, behaviorPropertyTemplate),
    distanceMatchBehaviorProperty: merge(distanceMatchBehaviorData, behaviorPropertyTemplate),

    //State Node----------------------------------------------------------------------------------------
    stateType: {
      name: "State",
      type: TYPES.OBJECT,
      instanceBlock: {
        onCanvas: true,
        color: "#7dd6ff",
        icon: ContainerIcon,
        connections: {
          bottom: {
            direction: CONNECTIONS.OUTBOUND,
            allowed: ["stateType", "powerOffType"],
            limitOne: false,
          },
          top: {
            direction: CONNECTIONS.INBOUND,
            allowed: ["stateType", "operationType", "programType", "powerOnType"],
            limitOne: false,
          },
        },
        extras: [
          {
            icon: FiMoreHorizontal,
            type: EXTRA_TYPES.DROPDOWN,
            contents: [
              EXTRA_TYPES.NAME_EDIT_TOGGLE,
              EXTRA_TYPES.COLLAPSE_TOGGLE,
              {
                type: EXTRA_TYPES.FUNCTION_BUTTON,
                icon: IoArrowRedo,
                onClick: 'teleport',
                label: "Teleport Here"
              },
              EXTRA_TYPES.DELETE_BUTTON,
              // EXTRA_TYPES.DEBUG_TOGGLE,
              // EXTRA_TYPES.SELECTION_TOGGLE,
            ],
          },
        ],
        hideNewPrefix: true,
      },
      properties: {
        children: {
          name: "Children",
          accepts: [
            "groupType",
            "positionMatchBehaviorProperty",
            "orientationMatchBehaviorProperty",
            "positionLivelinessBehaviorProperty",
            "orientationLivelinessBehaviorProperty",
            "positionMirroringBehaviorProperty",
            "orientationMirroringBehaviorProperty",
            "positionBoundingBehaviorProperty",
            "orientationBoundingBehaviorProperty",
            "jointMatchBehaviorProperty",
            "jointLivelinessBehaviorProperty",
            "jointMirroringBehaviorProperty",
            "jointLimitsBehaviorProperty",
            "jointBoundingBehaviorProperty",
            "collisionAvoidanceBehaviorProperty",
            "velocityMinimizationBehaviorProperty",
            "accelerationMinimizationBehaviorProperty",
            "jerkMinimizationBehaviorProperty",
            "originVelocityMinimizationBehaviorProperty",
            "originAccelerationMinimizationBehaviorProperty",
            "originJerkMinimizationBehaviorProperty",
            "relativeMotionLivelinessBehaviorProperty",
            "originPositionLivelinessBehaviorProperty",
            "originOrientationLivelinessBehaviorProperty",
            "originPositionMatchBehaviorProperty",
            "originOrientationMatchBehaviorProperty",
            "gravityBehaviorProperty",
            "smoothnessMacroBehaviorProperty",
            "distanceMatchBehaviorProperty"
          ],
          default: [],
          isList: true,
          fullWidth: true,
        },
      },
    },
    groupType: {
      name: "Group",
      type: TYPES.OBJECT,
      instanceBlock: {
        onCanvas: false,
        color: "#444444",
        icon: ContainerIcon,
        extras: [
          {
            icon: FiMoreHorizontal,
            type: EXTRA_TYPES.DROPDOWN,
            contents: [
              EXTRA_TYPES.NAME_EDIT_TOGGLE,
              EXTRA_TYPES.COLLAPSE_TOGGLE,
              EXTRA_TYPES.DELETE_BUTTON
            ],
          },
        ],
        hideNewPrefix: true,
      },
      properties: {
        priority: {
          name: "Priority",
          type: SIMPLE_PROPERTY_TYPES.OPTIONS,
          options:[
            {label:'Low',value:0.5},
            {label:'Standard',value:1},
            {label:'High',value:5},
            {label:'Critical',value:50}
          ],
          default: 1
        },
        children: {
          name: "Children",
          accepts: ["positionMatchBehaviorProperty",
            "orientationMatchBehaviorProperty",
            "positionLivelinessBehaviorProperty",
            "orientationLivelinessBehaviorProperty",
            "positionMirroringBehaviorProperty",
            "orientationMirroringBehaviorProperty",
            "positionBoundingBehaviorProperty",
            "orientationBoundingBehaviorProperty",
            "jointMatchBehaviorProperty",
            "jointLivelinessBehaviorProperty",
            "jointMirroringBehaviorProperty",
            "jointLimitsBehaviorProperty",
            "jointBoundingBehaviorProperty",
            "collisionAvoidanceBehaviorProperty",
            "velocityMinimizationBehaviorProperty",
            "accelerationMinimizationBehaviorProperty",
            "jerkMinimizationBehaviorProperty",
            "originVelocityMinimizationBehaviorProperty",
            "originAccelerationMinimizationBehaviorProperty",
            "originJerkMinimizationBehaviorProperty",
            "relativeMotionLivelinessBehaviorProperty",
            "originPositionLivelinessBehaviorProperty",
            "originOrientationLivelinessBehaviorProperty",
            "originPositionMatchBehaviorProperty",
            "originOrientationMatchBehaviorProperty",
            "gravityBehaviorProperty",
            "smoothnessMacroBehaviorProperty",
            "distanceMatchBehaviorProperty"
          ],
          default: [],
          isList: true,
          fullWidth: true,
        },
      },
    },
    //Power On Node--------------------------------------------------------------------------------------
    powerOnType: {
      name: "PowerOn",
      type: TYPES.OBJECT,
      instanceBlock: {
        onCanvas: true,
        color: "#eb4444",
        icon: ContainerIcon,
        connections: {
          bottom: {
            direction: CONNECTIONS.OUTBOUND,
            allowed: ["stateType","powerOffType"],
            limitOne: false,
          },
        },
        extras: [
          {
            type: EXTRA_TYPES.FUNCTION_BUTTON,
            icon: IoArrowRedo,
            onClick: 'teleport',
            label: "Teleport Here"
          }
        ],
        hideNewPrefix: true,
      },
    },
    //Power Off Node-------------------------------------------------------------------------------------
    powerOffType: {
      name: "Power Off",
      type: TYPES.OBJECT,
      instanceBlock: {
        onCanvas: true,
        color: "#7d746f",
        icon: ContainerIcon,
        connections: {
          top: {
            direction: CONNECTIONS.INBOUND,
            allowed: ["stateType", "powerOnType"],
            limitOne: false,
          },
        },
        extras: [
          {
            type: EXTRA_TYPES.FUNCTION_BUTTON,
            icon: IoArrowRedo,
            onClick: 'teleport',
            label: "Teleport Here"
          }
        ],
        hideNewPrefix: true,
      },
    }
  },
};