/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/


import { useGLTF } from '@react-three/drei';
import Ur3Wrist2 from './wrist2.glb';

export default function Model(props) {
  const { nodes } = useGLTF(Ur3Wrist2);
  return [
    { type: 'raw', geometry: nodes.eSeries_UR3e.geometry, material: nodes.eSeries_UR3e.material, scale:[0.001, 0.001, 0.001], rotation:[Math.PI/2, 0, 0]  },
    { type: 'raw', geometry: nodes.eSeries_UR3e_036.geometry, material: nodes.eSeries_UR3e_036.material, scale:[0.001, 0.001, 0.001], rotation:[Math.PI/2, 0, 0]  },
    { type: 'raw', geometry: nodes.eSeries_UR3e_037.geometry, material: nodes.eSeries_UR3e_037.material, scale:[0.001, 0.001, 0.001], rotation:[Math.PI/2, 0, 0]  }
  ]
}

useGLTF.preload(Ur3Wrist2)