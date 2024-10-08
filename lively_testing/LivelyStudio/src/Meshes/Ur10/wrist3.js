/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/

import Ur10Wrist3 from './wrist3.glb';
import { useGLTF } from '@react-three/drei';

export default function Model(props) {

  const { nodes, materials } = useGLTF(Ur10Wrist3);
  return [{type:'raw',geometry:nodes['node-shape0-name'].geometry,material:materials['SWMaterial-2_003']} ]




}

useGLTF.preload(Ur10Wrist3)
