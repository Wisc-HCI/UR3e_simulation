

export function shape2item(shape, isCollision) {
    let item = {
        name: shape.name,
        frame: shape.frame,
        position: {
            x: shape.localTransform.translation[0],
            y: shape.localTransform.translation[1],
            z: shape.localTransform.translation[2]
        },
        rotation: {
            w: shape.localTransform.rotation[3],
            x: shape.localTransform.rotation[0],
            y: shape.localTransform.rotation[1],
            z: shape.localTransform.rotation[2]
        },
        color: isCollision ? { r: 100, g: 0, b: 0, a: 1 } : { r: 100, g: 100, b: 100, a: 1 },
        scale: { x: 1, y: 1, z: 1 },
        transformMode: "inactive",
        wireframe: isCollision
    };
    switch (shape.type) {
        case 'Box':
            item.shape = 'cube';
            item.scale = { x: shape.x, y: shape.y, z: shape.z };
            break;
        case 'Sphere':
            item.shape = 'sphere';
            item.scale = { x: shape.radius*2, y: shape.radius*2, z: shape.radius*2 };
            break;
        case 'Cylinder':
            item.shape = 'cylinder';
            item.scale = { x: shape.radius*2, y: shape.radius*2, z: shape.length };
            break;
        case 'Capsule':
            item.shape = 'capsule';
            item.scale = { x: shape.radius*2, y: shape.radius*2, z: shape.length };
            break;
        case 'Mesh':
            item.shape = shape.filename;
            item.scale = { x: shape.x, y: shape.y, z: shape.z };
            item.color = undefined;
            break;
        default:
            item.shape = 'box'
    }
    return item
}

export function state2tfs(state) {
    let tfs = {};
    // console.log(state.proximity);
    Object.entries(state.frames).forEach(pair=>{
        tfs[pair[0]] = {
            frame: 'world',
            position: { 
                x: pair[1].world.translation[0], 
                y: pair[1].world.translation[1], 
                z: pair[1].world.translation[2] 
            },
            rotation: { 
                w: pair[1].world.rotation[3], 
                x: pair[1].world.rotation[0], 
                y: pair[1].world.rotation[1], 
                z: pair[1].world.rotation[2] 
            },
            scale: {
                x: 1,
                y: 1,
                z: 1
            }
        }
        tfs[pair[0]+'-translation'] = {
            frame: 'world',
            position: { 
                x: pair[1].world.translation[0], 
                y: pair[1].world.translation[1], 
                z: pair[1].world.translation[2] 
            },
            rotation: { 
                w: 1, 
                x: 0, 
                y: 0, 
                z: 0
            },
            scale: {
                x: 1,
                y: 1,
                z: 1
            }
        }
    })
    // console.log(tfs)
    return tfs
}

export function state2Lines(state) {
    let lines = {};
    state.proximity.forEach(({shape1,shape2,distance,points,loss})=>{
        // if (distance < 1 && (shape1 === 'Table' || shape2 === 'Table')) {
        //     console.log(`${shape1},${shape2},${distance},${loss}`)
        // }
        if (points) {
            // console.log(points)
            lines[`${shape1}__${shape2}`] = {
                name: "",
                frame: "world",
                width: 1,
                vertices: [
                    { position: { x: points[0][0], y: points[0][1], z: points[0][2] }, color: { r: 255, g: 0, b: 0, a: 1/Math.pow(Math.E,distance) } },
                    { position: { x: points[1][0], y: points[1][1], z: points[1][2] }, color: { r: 255, g: 0, b: 0, a: 1/Math.pow(Math.E,distance) } }],
                highlighted: false
            }
        }
        
    })
    return lines
}