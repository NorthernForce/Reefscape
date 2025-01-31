import { FormControlLabel, Switch, Tab, Tabs } from '@mui/material';
import './App.css';
import { useState } from 'react';
import { useEntry } from '@frc-web-components/react';
import Teleop from './teleop/Teleop';

function TabPanel(props: { children?: React.ReactNode, selected: number, index: number }) {
    return <div hidden={props.selected !== props.index}>
        {props.children}
    </div>
}

function App(props: { targetIp: string }) {
    let [selected, setSelected] = useState(0);
    let [tabEntry] = useEntry('FWC/selectedTab', 0)
    const handleTabChange = (_event: React.ChangeEvent<{}>, newValue: number) => {
        if (tabsLocked) return;
        setSelected(newValue);
    }
    let [tabsLocked, setTabsLocked] = useState(true);
    const handleLockChange = (_event: React.ChangeEvent<HTMLInputElement>, checked: boolean) => {
        setTabsLocked(checked);
    }
    let [connected] = useEntry('/FWC/connected', false);
    return (
        <>
            <div className="header">
                <Tabs id="header-tabs" value={tabsLocked ? tabEntry : selected} onChange={handleTabChange}>
                    <Tab label="Teleop" />
                    <Tab label="Autonomous" />
                    <Tab label="Settings" />
                </Tabs>
                <span className="header-status"
                    style={{ color: connected ? "green" : "#b5e349" }}>{connected ? "Connected to " : "Connecting to"} {props.targetIp}
                </span>
                <FormControlLabel control={<Switch id="lock-switch" value={tabsLocked} defaultChecked
                    onChange={handleLockChange}/>} label="Lock tabs"/>
            </div>
            <div>
                <TabPanel selected={tabsLocked ? tabEntry : selected} index={0}>
                    <Teleop />
                </TabPanel>
                <TabPanel selected={tabsLocked ? tabEntry : selected} index={1}>
                    Autonomous
                </TabPanel>
                <TabPanel selected={tabsLocked ? tabEntry : selected} index={2}>
                    Settings
                </TabPanel>
            </div>
        </>
    );
}

export default App;
