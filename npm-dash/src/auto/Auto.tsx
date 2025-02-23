import { Field, FieldPath, FieldRobot, NetworkAlerts, useEntry } from "@frc-web-components/react";
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
    const [autoPose] = useEntry('/FWC/AutoPose', [0, 0, 0]);
    const [autoPath] = useEntry('/FWC/AutoPath', [0, 0, 2, 2]);
    const [choreoAlertsWarning] = useEntry('/SmartDashboard/Choreo Alerts/warnings', []);
    const [choreoAlertsError] = useEntry('/SmartDashboard/Choreo Alerts/errors', []);
    const [choreoAlertsInfo] = useEntry('/SmartDashboard/Choreo Alerts/infos', []);
    const [alertsWarning] = useEntry('/SmartDashboard/Alerts/warnings', []);
    const [alertsError] = useEntry('/SmartDashboard/Alerts/errors', []);
    const [alertsInfo] = useEntry('/SmartDashboard/Alerts/infos', []);
    const combinedInfo = [...choreoAlertsInfo, ...alertsInfo];
    const combinedWarning = [...choreoAlertsWarning, ...alertsWarning];
    const combinedError = [...choreoAlertsError, ...alertsError];
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
                    <NetworkAlerts className="auto-alerts" infos={combinedInfo} warnings={combinedWarning}
                        errors={combinedError} />
                </div>
            </div>
        </>
    );
}

export default Auto;