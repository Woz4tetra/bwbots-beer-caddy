import { useState } from "react";

export default function WaypointsPage(props: any) {
    const { waypoints, setWaypoints } = props;

    const [ name, setName ] = useState("");
    const [ x, setX ] = useState("");
    const [ y, setY ] = useState("");
    const [ th, setTh ] = useState("");

    const waypointsComponent = waypoints.map((waypoint: any) => 
        <div>{waypoint.name}: <span>({waypoint.x}, {waypoint.y}, {waypoint.th})</span></div>
    );

    const onNameChange = (e: any) => setName(e.target.value);
    const onXChange = (e: any) => setX(e.target.value);
    const onYChange = (e: any) => setY(e.target.value);
    const onThChange = (e: any) => setTh(e.target.value);

    const onAddWaypoint = (e: any) => {
        e.preventDefault();

        if (name === "" || x === "" || y === "" || th === "") { return; }

        setWaypoints([
            ...waypoints,
            { 
                name: name,
                x: x,
                y: y,
                th: th
            }
        ]);
        
        setName("");
        setX("");
        setY("");
        setTh("");
    }

    return <div>
        {waypointsComponent}
        <div>Living Room: <span>(0, 0, 90)</span></div>
        <div>Bedroom 1: <span>(12, 24, 180)</span></div>
        <div>
            <h1>New Waypoint</h1>
            <form>
                <label>Waypoint name: </label>
                <input 
                    type="text"
                    value={name}
                    onChange={onNameChange}
                />
                <br/>
                <label>X: </label>
                <input 
                    type="text"
                    value={x}
                    onChange={onXChange}
                />
                <br/>
                <label>Y: </label>
                <input 
                    type="text"
                    value={y}
                    onChange={onYChange}
                />
                <br/>
                <label>Th: </label>
                <input 
                    type="text"
                    value={th}
                    onChange={onThChange}
                />
                <br/>
                <button onClick={onAddWaypoint}>
                    {"Add Waypoint"}
                </button>
            </form>
        </div>
    </div>
}