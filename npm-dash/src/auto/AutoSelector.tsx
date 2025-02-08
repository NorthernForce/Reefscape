import { List, ListItem, ListItemButton, ListItemText } from "@mui/material";

import './AutoSelector.css';
import { useEntry } from "@frc-web-components/react";

function AutoSelector(props: { source: string }) {
    let [options] = useEntry(props.source + '/options', ['No Options Found']);
    let [activeChoice] = useEntry(props.source + '/active', '');
    let [_selectedChoice, setSelectedChoice] = useEntry(props.source + '/selected', '');

    return (
        <>
            <div className="auto-selector" >
                <List>
                    {options.map((option: string, index: number) => {
                        return <ListItem key={index}>
                            <ListItemButton selected={option == activeChoice} onClick={() => setSelectedChoice(option)}>
                                <ListItemText primary={option} />
                            </ListItemButton>
                        </ListItem>
                    })}
                </List>
            </div>
        </>
    );
}

export default AutoSelector;