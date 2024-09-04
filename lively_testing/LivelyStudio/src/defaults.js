
export const DEFAULTS = {
  urdf:'<xml></xml>',
  rootBounds: [
    { value: 0.0, delta: 0.0 }, { value: 0.0, delta: 0.0 }, { value: 0.0, delta: 0.0 }, // Translational
    { value: 0.0, delta: 0.0 }, { value: 0.0, delta: 0.0 }, { value: 0.0, delta: 0.0 }  // Rotational
  ],
  persistentShapes: [],
  shapeUpdates: [],
  objectives: [],
  goals: [],
  weights: [],
  box: { type: 'Box', frame: 'world', name: 'New Box', physical: true, x: 0, y: 0, z: 0, localTransform: { translation: [0, 0, 0], rotation: [1, 0, 0, 0] } },
  cylinder: { type: 'Cylinder', frame: 'world', name: 'New Cylinder', physical: true, length: 0.1, radius: 0.05, localTransform: { translation: [0, 0, 0], rotation: [1, 0, 0, 0] } },
  capsule: { type: 'Capsule', frame: 'world', name: 'New Capsule', physical: true, length: 0.1, radius: 0.05, localTransform: { translation: [0, 0, 0], rotation: [1, 0, 0, 0] } },
  sphere: { type: 'Sphere', frame: 'world', name: 'New Sphere', physical: true, radius: 0.05, localTransform: { translation: [0, 0, 0], rotation: [1, 0, 0, 0] } }
}