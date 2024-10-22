/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/


import { useGLTF } from '@react-three/drei';
import TorsoMesh from './Torso.glb';

export default function Model(props) {

  const { nodes, materials } = useGLTF(TorsoMesh);
  return [{type:'raw', geometry:nodes.Torso.geometry, material:materials.TorsoUV, scale:[0.01, 0.01, 0.01]}]



}

useGLTF.preload(TorsoMesh)
