/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/


import { useGLTF } from '@react-three/drei';
import LocationMarker from './LocationMarker.glb'

export default function Model(props) {

  const { nodes } = useGLTF(LocationMarker)
  return [
    {
      type: 'group',
      children: [
        {
          type: 'raw',
          geometry: nodes.LocationMarker.geometry,
          material: nodes.LocationMarker.material
        }
      ]
    }
  ]
}

useGLTF.preload(LocationMarker)
