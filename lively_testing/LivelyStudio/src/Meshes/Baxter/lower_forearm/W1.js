/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/


import { useGLTF } from '@react-three/drei';
import W1 from './W1.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(W1);


  return [{type:'group', children: [{type:'raw', geometry:nodes.W1_1.geometry, material:materials['Material_001.004']},
                             {type:'raw',geometry:nodes.W1_2.geometry, material:materials['Material_002.001']}], rotation:[-Math.PI, 0, 0] }]
}

useGLTF.preload(W1)
