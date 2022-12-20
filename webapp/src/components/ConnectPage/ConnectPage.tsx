import { useState } from "react"

export default function ConnectPage(props: any) {
    const { rosClient, setRosClient } = props;
    const [robotIP, setRobotIP] = useState("");

    const onRobotIPChange = (e: any) => setRobotIP(e.target.value);
    
    const onRobotConnect = (e: any) => {
        e.preventDefault();

        var ROSLIB = require('roslib');

        const ros = new ROSLIB.Ros({
            url: robotIP
        });
        
        ros.on('connection', () => {
            console.log('Connected to ROS websocket server');
            setRosClient(ros);
        });

        ros.on('error', (error: any) => {
            console.log('Error connecting to websocket server: ', error);
            setRosClient(undefined);
        });
        
        ros.on('close', () => {
            console.log('Connection to websocket server closed.');
            setRosClient(undefined);
        });
    }

    const onRobotDisconnect = (e: any) => {
        e.preventDefault()
        rosClient.close();
    }

    return <div>
        <div>
            <h1>Robot</h1>
            <form>
                <label>IP: </label>
                <input 
                    type="text"
                    value={robotIP}
                    onChange={onRobotIPChange}
                />
                <br/>
                <button onClick={!rosClient ? onRobotConnect : onRobotDisconnect}>
                    {!rosClient ? "Connect Robot" : "Disconnect Robot"}
                </button>
            </form>
        </div>
    </div>
}