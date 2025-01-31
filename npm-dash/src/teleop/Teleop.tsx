import { Canvas, CanvasMjpgStream, NetworkAlerts, useEntry } from "@frc-web-components/react";
import "./Teleop.css"

function TimeDisplay(props: { time: number }) {
    let minutes = Math.floor(props.time / 60);
    let seconds = Math.floor(props.time % 60);
    let secondsString = seconds < 10 ? `0${seconds}` : `${seconds}`;
    return <div className="time-display">
        {minutes}:{secondsString}
    </div>
}

function Teleop() {
    let [time] = useEntry("/FMSInfo/MatchTime", 0);
    let remainingTime = () => 150 - time;

    return <>
        <div className="teleop-container">
            <NetworkAlerts source-key="/Alerts" />
            <div className="time-display">
                <TimeDisplay time={remainingTime()} />
            </div>
            <Canvas className="camera-feed">
                <CanvasMjpgStream origin={[0,0]} crosshairColor="white" srcs={["http://10.1.72.15:1183/stream.mjpg"]} />
            </Canvas>
            <div></div>
        </div>
    </>
}

export default Teleop;