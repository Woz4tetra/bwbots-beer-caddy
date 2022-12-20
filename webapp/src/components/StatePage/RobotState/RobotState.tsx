import { useEffect, useState } from "react";

export default function RobotState(props: any) {
    const { rosClient } = props;
    const [robotState, setRobotState] = useState([0, 0, 0]);

    useEffect(() => {
        if (!rosClient) return;
        
        var ROSLIB = require('roslib');

        var odomListener = new ROSLIB.Topic({
            ros: rosClient,
            name: '/bw/odom',
            messageType: 'nav_msgs/Odometry'
        });

        const odomCallback = (msg: any) => {
            setRobotState([
                Math.round(Number(msg.pose.pose.position.x) * 100) / 100,
                Math.round(Number(msg.pose.pose.position.y) * 100) / 100,
                Math.round(Number(msg.twist.twist.angular.z) * 100) / 100
            ])
        };

        odomListener.subscribe(odomCallback);

        console.log("Subscribed to robot topics.")

        return () => {
            odomListener.unsubscribe(odomCallback);
            console.log("Unsubscribed from robot topics.")
        };
    }, [rosClient]);

    return (
    <div>
        <h1>Robot</h1>
        <div>State: <span>{robotState[0] + ', ' + robotState[1] + ', ' + robotState[2]}</span></div>
        <div>Battery: <span>N/A</span></div>
        <div>Connection: <span>Disconnected</span></div>
    </div>
    );
}