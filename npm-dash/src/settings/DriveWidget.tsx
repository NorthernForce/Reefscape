import { RobotCommand } from '@frc-web-components/react';
import './DriveWidget.css';
function DriveWidget()
{
    return (
        <div className="drive-widget">
            <RobotCommand className="reset-encoders" name="Reset Encoders" source-key="/FWC/swerve/resetEncoders" />
        </div>
    );
}
export default DriveWidget;