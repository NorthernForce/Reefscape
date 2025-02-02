import { Field, FieldPath, FieldRobot, useEntry } from "@frc-web-components/react";
import AutoSelector from "./AutoSelector";

import './Auto.css';
import { fieldConfigs } from "@frc-web-components/fwc";

fieldConfigs.push(
    {
      game: 'Reefscape',
      image: './field-images/2025-field.jpg',
      corners: {
        topLeft: [425, 88],
        bottomRight: [3348, 1445],
      },
      size: [57.5721785, 26.417323],
      unit: 'foot',
    }
)

function Auto() {
    let [autoPose] = useEntry('/SmartDashboard/AutoPose', [0, 0, 0]);
    let [autoPath] = useEntry('/SmartDashboard/AutoPath', [0, 0, 2, 2]);
    return (
        <>
            <div className="auto-container">
                <div className="auto-selector-container">
                    <AutoSelector source="/SmartDashboard/AutoChooser"/>
                </div>
                <div className="auto-field-container">
                    <Field game="Reefscape" className="auto-field">
                        <FieldRobot pose={autoPose} />
                        <FieldPath translations={autoPath} />
                    </Field>
                </div>
            </div>
        </>
    );
}

export default Auto;