/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/

import { useGLTF } from '@react-three/drei'
import mesh from './LFinger11.glb'

export default function Model() {
  const { nodes, materials } = useGLTF(mesh)

  return [{
    type : 'group', children : [{type : 'raw' , geometry : nodes.imagetostl_mesh.geometry, material : materials.mat0 }]
  }]


  
     
}

useGLTF.preload(mesh)
