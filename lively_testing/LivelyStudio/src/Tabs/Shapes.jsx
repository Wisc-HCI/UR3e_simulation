import React, { useState } from "react";
import { useCallback } from "react";
import {
    Box, Heading, Header, Card, CardBody, CardHeader,
    Grid, Text, Avatar, Stack, List, Collapsible, Button,
    DropButton, Menu
} from "grommet";
import { Lock, CaretRightFill, CaretDownFill, Add } from 'grommet-icons';
import useStore from "../store";
import { ReactComponent as Cube } from '../Icons/Cube.svg';
import { ReactComponent as Cylinder } from '../Icons/Cylinder.svg';
import { ReactComponent as Sphere } from '../Icons/Sphere.svg';
import { ReactComponent as Capsule } from '../Icons/Capsule.svg';
import { PositionField } from '../Fields/Position';
import { RotationField } from '../Fields/Rotation';
import { CylindricalField } from '../Fields/Cylindrical';
import { SphericalField } from '../Fields/Spherical';
import { SizeField } from '../Fields/Size';

const ShapeLookup = { Cube, Cylinder, Sphere, Capsule };

export const ShapeCard = ({ urdf, link, idx, listIdx }) => {
    const shapeInfo = useStore(useCallback(store => {
        let shape = { type: 'null', name: 'null', localTransform: { translation: [0, 0, 0], rotation: [0, 0, 0, 1] } };
        if (urdf) {
            shape = store.links.filter(ulink => ulink.name === link)[0]?.collisions[idx]
        } else {
            shape = store.persistentShapes[idx]
        }
        return shape
    }, [urdf, link, idx]))
    console.log(shapeInfo)
    const [shapeExpanded, setShapeExpanded] = useState(false);
    const [positionExpanded, setPositionExpanded] = useState(false);
    const [rotationExpanded, setRotationExpanded] = useState(false);
    const [sizeExpanded, setSizeExpanded] = useState(false);

    const ShapeIcon = ShapeLookup[shapeInfo.type]

    return (
        <Card
            elevation='none'
            animation={{ type: 'fadeIn', delay: listIdx * 20 }}
            focusIndicator={false}
            hoverIndicator={true}
        >
            <CardHeader background={{ color: urdf ? 'accent-1' : 'accent-3', opacity: 'strong' }} pad={{ horizontal: 'medium', vertical: 'small' }}>
                <Stack anchor="top-right">
                    <Avatar background='white'>
                        <ShapeIcon style={{ width: 30 }} />
                    </Avatar>
                    {urdf && (
                        <Box pad="xxsmall" round background="accent-4" responsive={false}><Lock size='small' /></Box>
                    )}
                </Stack>
                <Text>{shapeInfo.type}</Text>
                <Button icon={shapeExpanded ? <CaretDownFill /> : <CaretRightFill />} onClick={() => setShapeExpanded(!shapeExpanded)} />
            </CardHeader>
            <Collapsible direction='vertical' open={shapeExpanded} >
                <CardBody background='background-contrast' pad='small' gap='xsmall' >
                    <Box round='small' background='background-contrast' pad='small' gap='xsmall' alignContent='between'>
                        <Header>
                            <Text>Position</Text>
                            <Button size='small' icon={positionExpanded ? <CaretDownFill /> : <CaretRightFill />} onClick={() => setPositionExpanded(!positionExpanded)} />
                        </Header>
                        <Collapsible direction='vertical' open={positionExpanded}>
                            <PositionField
                                x={shapeInfo.localTransform.translation[0]}
                                y={shapeInfo.localTransform.translation[1]}
                                z={shapeInfo.localTransform.translation[2]}
                                onChange={(v) => { console.log(v) }}
                                disabled={urdf}
                            />
                        </Collapsible>
                    </Box>
                    <Box round='small' background='background-contrast' pad='small' gap='xsmall' alignContent='between'>
                        <Header>
                            <Text>Rotation</Text>
                            <Button size='small' icon={rotationExpanded ? <CaretDownFill /> : <CaretRightFill />} onClick={() => setRotationExpanded(!rotationExpanded)} />
                        </Header>
                        <Collapsible direction='vertical' open={rotationExpanded}>
                            <RotationField
                                w={shapeInfo.localTransform.translation[3]}
                                x={shapeInfo.localTransform.translation[0]}
                                y={shapeInfo.localTransform.translation[1]}
                                z={shapeInfo.localTransform.translation[2]}
                                onChange={(v) => { console.log(v) }}
                                disabled={urdf}
                            />
                        </Collapsible>
                    </Box>

                    <Box round='small' background='background-contrast' pad='small' gap='xsmall' alignContent='between'>
                        <Header>
                            <Text>Size</Text>
                            <Button size='small' icon={sizeExpanded ? <CaretDownFill /> : <CaretRightFill />} onClick={() => setSizeExpanded(!sizeExpanded)} />
                        </Header>
                        <Collapsible direction='vertical' open={sizeExpanded}>
                            {shapeInfo.type === 'Box' && (
                                <SizeField
                                    x={shapeInfo.x}
                                    y={shapeInfo.y}
                                    z={shapeInfo.z}
                                    onChange={(v) => { console.log(v) }}
                                    disabled={urdf}
                                />
                            )}
                            {(shapeInfo.type === 'Cylinder' || shapeInfo.type === 'Capsule') && (
                                <CylindricalField
                                    length={shapeInfo.length}
                                    radius={shapeInfo.radius}
                                    onChange={(v) => { console.log(v) }}
                                    disabled={urdf}
                                />
                            )}
                            {shapeInfo.type === 'Sphere' && (
                                <SphericalField
                                    radius={shapeInfo.radius}
                                    onChange={(v) => { console.log(v) }}
                                    disabled={urdf}
                                />
                            )}
                        </Collapsible>
                    </Box>
                </CardBody>
            </Collapsible>
        </Card>
    )
}

export const LinkPreview = ({ id, idx }) => {
    const focusItem = useStore(store => store.focusItem);
    const setFocusItem = useStore(store => store.setFocusItem);
    const clearFocusItem = useStore(store => store.clearFocusItem);
    const linkData = useStore(store => {
        if (id === 'world') {
            return { name: 'world', collisions: [] }
        } else if (store.links[id]) {
            return store.links[id]
        } else {
            return { name: 'null', collisions: [] }
        }
    })

    const additionalShapes = useStore(useCallback(store => store.persistentShapes.filter(d => d.frame === linkData.name), [linkData]));

    return (
        <Card
            key={linkData.name}
            animation={[{ type: 'fadeIn', delay: idx * 20 }]}
            focusIndicator={false}
            style={{ boxShadow: focusItem?.type === 'link' && focusItem?.item === id ? '0pt 0pt 5pt 5pt #CCCCCC99' : null }}
            hoverIndicator={true}
            elevation="none"
            onClick={() => {
                if (focusItem === null || focusItem?.type !== 'link' || (focusItem?.type === 'link' && focusItem?.item !== id)) { setFocusItem('link', id) } else { clearFocusItem() };
            }}
        >
            <CardHeader background={{ color: 'brand', opacity: 'strong' }} pad={{ horizontal: 'medium', vertical: 'small' }}>
                <Text style={{ textTransform: 'capitalize' }}>{linkData.name.replaceAll('_', ' ')}</Text>
            </CardHeader>
            <CardBody background='brand' pad="small">
                {linkData.collisions.length} Shape{linkData.collisions.length !== 1 ? "s" : ""} from URDF<br />
                {additionalShapes.length > 0 && (
                    <Text>{additionalShapes.length} Custom Shape{additionalShapes.length !== 1 ? "s" : ""}</Text>
                )}
            </CardBody>
        </Card>
    )
}

export const LinkDetail = () => {
    const focusItem = useStore(store => store.focusItem);
    const setFocusItem = useStore(store => store.setFocusItem);
    const addPersistentShape = useStore(store => store.addPersistentShape);
    const linkData = useStore(store => {
        if (focusItem.item === 'world') {
            return { name: 'world', collisions: [] }
        } else if (store.links[focusItem.item]) {
            return store.links[focusItem.item]
        } else {
            return { name: 'null', collisions: [] }
        }
    })

    const urdfShapes = linkData.collisions.map((collider, i)=>({...collider,index:i, urdf:true}));
    const additionalShapes = useStore(useCallback(store => store.persistentShapes.map((shape, i) => ({ ...shape, index: i, urdf:false })).filter(d => d.frame === linkData.name), [linkData]));

    return (
        <Box fill>
            <Header level={5} style={{ textTransform: 'capitalize', paddingLeft: 20 }}>
                {linkData.name.replaceAll('_', ' ')}
                <Menu
                    plain
                    items={[
                        { label: 'New Box', onClick: () => addPersistentShape(linkData.name,'box') },
                        { label: 'New Sphere', onClick: () => addPersistentShape(linkData.name,'sphere') },
                        { label: 'New Cylinder', onClick: () => addPersistentShape(linkData.name,'cylinder') },
                        { label: 'New Capsule', onClick: () => addPersistentShape(linkData.name,'capsule') },
                    ]}
                    dropProps={{ elevation: 'none' }}
                >
                    {({ disabled, drop, hover }) => {
                        const color = hover && !drop && !disabled ? 'accent-1' : undefined;
                        return (
                            <Box
                                elevation='none'
                                direction="row"
                                gap="small"
                                pad="small"
                                background={hover && drop ? 'light-2' : undefined}
                            >
                                <Add color={color} />
                            </Box>
                        );
                    }}
                </Menu>
            </Header>
            <List data={[...urdfShapes,...additionalShapes]} border={false} style={{ overflowY: 'scroll' }} margin='none'>
                {(item,i) => (
                    <ShapeCard urdf={item.urdf} link={item.name} idx={item.index} key={item.index} listIdx={i} />
                )}
            </List>
        </Box>
    )
}

export const Shapes = () => {
    const robotLinkKeys = useStore(store => (['world', ...store.links.map((_, i) => i)]));

    return (
        <>
            <Box direction='row' justify='between' alignContent='center'>
                <Heading level={4} margin={{ horizontal: 'small' }}>Collision Shapes</Heading>
            </Box>
            <Box fill pad="large" height="100%" background='black'>
                <Grid fill gap="medium" columns={{ count: 'fit', size: 'small' }}>
                    {robotLinkKeys.map((id, idx) => <LinkPreview id={id} key={id} idx={idx} />)}
                </Grid>
            </Box>
        </>
    )
}