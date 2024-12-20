/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/
import { useGLTF } from '@react-three/drei';
import RFinger13Mesh from './RFinger13.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(RFinger13Mesh);
  return [{type:'raw', geometry:nodes.RFinger13.geometry,material:materials.RFinger13UV,scale:[0.01, 0.01, 0.01]}]
}

useGLTF.preload(RFinger13Mesh)
