/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/


import { useGLTF } from '@react-three/drei';
import Benchy from './3DBenchy.glb';

export default function Model(props) {

  const { nodes } = useGLTF(Benchy);
  return [{type:'raw', geometry:nodes['3DBenchy'].geometry, material:nodes['3DBenchy'].material} ]




}

useGLTF.preload(Benchy)
