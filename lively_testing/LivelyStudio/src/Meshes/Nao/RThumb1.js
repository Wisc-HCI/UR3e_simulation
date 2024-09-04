/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/


import { useGLTF } from '@react-three/drei';
import RThumb1Mesh from './RThumb1.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(RThumb1Mesh);
  return [{type:'raw', geometry:nodes.RThumb1.geometry, material:materials.RThumb1UV, scale:[0.01, 0.01, 0.01]}]
}

useGLTF.preload(RThumb1Mesh)
