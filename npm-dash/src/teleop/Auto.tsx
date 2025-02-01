import { Field } from "@frc-web-components/react";
import AutoSelector from "./AutoSelector";

import './Auto.css';

function Auto() {
    return (
        <>
            <div className="auto-container">
                <div className="auto-selector-container">
                    <AutoSelector source="/AutoChooser"/>
                </div>
                <div className="auto-field">
                    <Field game="Reefscape" />
                </div>
            </div>
        </>
    );
}

export default Auto;