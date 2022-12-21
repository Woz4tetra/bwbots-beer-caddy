import { useEffect, useState } from "react";

export default function DeliverPage(props: any) {
    const { rosClient } = props;

    const [ runBehaviorClient, setRunBehaviorClient ] = useState<any>(null);
    const [ addMissionTopic, setAddMissionTopic ] = useState<any>(null);

    const [ dispenserTag, setDispenserTag ] = useState('');
    const [ waypointName, setWaypointName ] = useState('');

    // setup publisher/action server
    useEffect(() => {
        if (!rosClient) return;

        var ROSLIB = require('roslib');

        const actionClient = new ROSLIB.ActionClient({
            ros: rosClient,
            serverName: '/bw/run_behavior',
            actionName: 'bw_interfaces/RunBehaviorAction'
        });
        setRunBehaviorClient(actionClient);

        const missionTopic = new ROSLIB.Topic({
            ros: rosClient,
            name: '/bw/add_mission',
            messageType: 'bw_interfaces/DrinkMission'
        });
        setAddMissionTopic(missionTopic);

        return () => {
            setRunBehaviorClient(null);
            setAddMissionTopic(null);
        }
    },[rosClient]);

    // handle inputs
    const onDispenserTagChange = (e: any) => setDispenserTag(e.target.value);
    const onWaypointNameChange = (e: any) => setWaypointName(e.target.value);

    // deliver button callback
    const onDeliverPress = (e: any) => {
        e.preventDefault();

        if (!addMissionTopic || !runBehaviorClient) return;
        if (waypointName === '' || dispenserTag === '') return;

        var ROSLIB = require('roslib');

        // send waypoint to /bw/add_mission
        var mission = new ROSLIB.Message({
            delivery_waypoint: waypointName,
            drink_dispenser_waypoint: dispenserTag
        });
        addMissionTopic.publish(mission);
        console.log('Mission sent');

        // call RunBehavior action server with 'drink_mission' behavior
        var goal = new ROSLIB.Goal({
            actionClient: runBehaviorClient,
            goalMessage: {
                behavior: 'drink_mission'
            }
        });

        goal.on('result', (r: any) => {
            console.log('Action result:', r.success);
        });

        goal.send();
        console.log('Action sent');
    }

    if (!rosClient) {
        return <div>Not connected to robot.</div>
    }

    return <div>
        <div>
            <form>
                <label>Dispenser Tag: </label>
                <input 
                    type="text"
                    value={dispenserTag}
                    onChange={onDispenserTagChange}
                />
                <br/>
                <label>Waypoint: </label>
                <input 
                    type="text"
                    value={waypointName}
                    onChange={onWaypointNameChange}
                />
                <br/>
                <button onClick={onDeliverPress}>
                    Deliver Now
                </button>
            </form>
        </div>
    </div>
}