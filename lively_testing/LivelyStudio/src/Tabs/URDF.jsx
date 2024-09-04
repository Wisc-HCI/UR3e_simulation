import React from "react";
import AceEditor from "react-ace";
import "ace-builds/src-noconflict/mode-xml";
import "ace-builds/src-noconflict/theme-chaos";
import useStore from "../store";
import { Box, Heading, Text, Menu } from "grommet";
import { StatusInfo, StatusGood, StatusWarning, FormDown } from 'grommet-icons';
import { panda } from "../DefaultRobots/panda";
import { ur3 } from "../DefaultRobots/ur3";
// import MonacoEditor from '@uiw/react-monacoeditor';
import Editor from "@monaco-editor/react";

const PresetMenu = (props) => {
    
    const setUrdf = useStore(store => store.setUrdf);

    return (
    <Menu
        plain
        items={[
            { label: 'Panda', onClick: () => setUrdf(panda) },
            { label: 'UR3e', onClick: () => setUrdf(ur3) },
        ]}
        dropProps={{elevation:'none'}}
        {...props}
    >
        {({ disabled, drop, hover, focus }) => {
            const color = hover && !drop && !disabled ? 'accent-1' : undefined;
            return (
                <Box
                    elevation='none'
                    direction="row"
                    gap="small"
                    pad="small"
                    background={hover && drop ? 'light-2' : undefined}
                >
                    <Text color={color}>{focus ? 'actions' : 'Presets'}</Text>
                    <FormDown color={color} />
                </Box>
            );
        }}
    </Menu>
)};

export const URDF = () => {

    const urdf = useStore(store => store.urdf);
    const setUrdf = useStore(store => store.setUrdf);
    const isValid = useStore(store => store.solver !== null && store.solver !== undefined);
    const isEmpty = urdf === '';
    const hasError = !isValid && !isEmpty;

    return (
        <>
            {/* <Box direction='row' justify='between' alignContent='center' animation='fadeIn'>
                <Heading level={4} margin={{ horizontal: 'small' }}>URDF Data</Heading>
                <Box
                    direction='row'
                    level={5}
                    style={{ fontStyle: 'italic', opacity: 0.5 }}
                    pad='small'
                    margin='small'
                    round='small'
                    background={isValid ? 'status-ok' : hasError ? 'status-critical' : 'status-unknown'}
                    gap='small'>
                    {isValid ? (
                        <StatusGood />
                    ) : hasError ? (
                        <StatusWarning />
                    ) : (
                        <StatusInfo />
                    )}
                    {isValid ? (
                        <Text>
                            URDF Valid!
                        </Text>
                    ) : hasError ? (
                        <Text>
                            URDF Invalid!
                        </Text>
                    ) : (
                        <Text>
                            Copy/Paste or Type URDF data below
                        </Text>
                    )}
                </Box>
                <PresetMenu />
            </Box> */}
            <Box animation='fadeIn' fill background='black'>
                <Editor
                    language='xml'
                    value={urdf}
                    onChange={(v,e)=>setUrdf(v)}
                    theme='vs-dark'
                />
            {/* <AceEditor
                width='100%'
                height='100%'
                mode="xml"
                theme="chaos"
                onChange={setUrdf}
                name="urdf_editor"
                showPrintMargin={false}
                value={urdf}
                editorProps={{ $blockScrolling: true }}
            /> */}
            </Box>
            
        </>

    )
}