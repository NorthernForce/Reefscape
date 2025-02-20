import { Gauge, RobotCommand, useEntry } from '@frc-web-components/react';
import './WristWidget.css';

function WristWidget() {
    const [angle] = useEntry("/FWC/Wrist/Angle", 0);
    return (
        <div className="wrist-widget-container">
            <RobotCommand className="reset-wrist" name="Reset Encoder" source-key="/SmartDashboard/ResetWrist" />
            <Gauge className="wrist-angle" value={angle} min={0} max={360} />
        </div>
    );
}

export default WristWidget;