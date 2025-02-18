import { Accordion, AccordionDetails, AccordionSummary } from '@mui/material';
import DriveWidget from './DriveWidget';
import './Settings.css';
import PoseWidget from './PoseWidget';
function Settings() {
    return (
        <div className="settings-container">
            <Accordion>
                <AccordionSummary>Drive</AccordionSummary>
                <AccordionDetails><DriveWidget /></AccordionDetails>
            </Accordion>
            <Accordion>
                <AccordionSummary>Pose</AccordionSummary>
                <AccordionDetails><PoseWidget /></AccordionDetails>
            </Accordion>
        </div>
    );
}
export default Settings;