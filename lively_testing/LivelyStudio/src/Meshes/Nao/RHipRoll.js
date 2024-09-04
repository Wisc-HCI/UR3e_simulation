/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/


import { useGLTF } from '@react-three/drei';
import RHipRollMesh from './RHipRoll.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(RHipRollMesh);
  return [{type:'raw', geometry:nodes.RHipRoll.geometry, material:materials.RHipRollUV, scale:[0.01, 0.01, 0.01]}]



}

useGLTF.preload(RHipRollMesh)
