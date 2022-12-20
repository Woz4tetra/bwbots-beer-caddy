import { useEffect, useState } from "react";

const DISPENSER_STATUS_URI = "https://192.168.43.1/status";
const TEST_REQUEST_URI = "https://api.nasa.gov/planetary/apod?api_key=DEMO_KEY";

export default function DispenserState() {
    const [bottleCount, setBottleCount] = useState("");
    const [connected, setConnected] = useState(false);

    // check for dispenser status every second
    // useEffect(() => {
    //     const axios = require('axios');

    //     const interval = setInterval(() => {
    //         axios.get(DISPENSER_STATUS_URI)
    //         .then((response: any) => {
    //             setConnected(true);
    //             setBottleCount(response.data.bottleCount);
    //         })
    //         .catch((error: any) => {
    //             setConnected(false);
    //             setBottleCount("who knows");
    //         });
    //     }, 1000);

    //     return () => clearInterval(interval);
    // }, []);

    return <div>
        <h1>Dispenser</h1>
        <div>Beer Stock: <span>{bottleCount} / 10</span></div>
        <div>Connection: <span>{connected ? "Connected" : "Not Connected"}</span></div>
    </div>;
}