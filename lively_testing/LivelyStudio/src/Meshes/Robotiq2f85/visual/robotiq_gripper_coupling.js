/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/


import { useGLTF } from '@react-three/drei';
import RobotiqVisualGripper from './robotiq_gripper_coupling.glb';
export default function Model(props) {
  const { nodes } = useGLTF(RobotiqVisualGripper);
  return [{type:'raw',geometry:nodes.Robotiq_Gripper_Coupling.geometry,material:nodes.Robotiq_Gripper_Coupling.material}]
}

useGLTF.preload(RobotiqVisualGripper)
