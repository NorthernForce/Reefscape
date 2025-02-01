import { List, ListItem, ListItemButton, ListItemText } from "@mui/material";

import './AutoSelector.css';
import { useEntry } from "@frc-web-components/react";

function AutoSelector(props: { source: string }) {
    let [options] = useEntry(props.source + "/options", []);
    let [defaultChoice] = useEntry(props.source + "/default", 0);
    let [activeChoice] = useEntry(props.source + "/active", defaultChoice);
    let [_selectedChoice, setSelectedChoice] = useEntry(props.source + "/selected", activeChoice);

    return (
        <>
            <div className="auto-selector" >
                <List>
                    {options.map((option: string, index: number) => {
                        return <ListItem key={index}>
                            <ListItemButton selected={activeChoice === index} onClick={() => setSelectedChoice(index)}>
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