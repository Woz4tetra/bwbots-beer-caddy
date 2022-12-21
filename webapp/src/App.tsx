import {
  BrowserRouter as Router,
  Navigate,
  Routes,
  Route,
} from "react-router-dom";
import './App.css';

import NavBar from './components/NavBar';

import { ConnectPage } from './components/ConnectPage';
import { DeliverPage } from "./components/DeliverPage";
import { WaypointsPage } from "./components/WaypointsPage";
import { StatePage } from "./components/StatePage";
import { useState } from "react";

// const pathRoot = "/beer-delivery-robot-webapp/";
const pathRoot = "/";

export function constructPath(path: string) {
  return pathRoot + path;
}

function App() {
  const [rosClient, setRosClient] = useState(undefined);
  const [waypoints, setWaypoints] = useState([]);

  const defaultProps = {
    rosClient: rosClient,
    setRosClient: setRosClient,
    waypoints: waypoints,
    setWaypoints: setWaypoints
  };

  return <Router>
    <NavBar />
    <Routes>
      <Route path={constructPath("connect")} element={<ConnectPage {...defaultProps} />}/>
      <Route path={constructPath("deliver")} element={<DeliverPage {...defaultProps} />}/>
      <Route path={constructPath("waypoints")} element={<WaypointsPage {...defaultProps} />}/>
      <Route path={constructPath("state")} element={<StatePage {...defaultProps} />}/>
      <Route path={constructPath("*")} element={<Navigate to={constructPath("connect")} />}/>
    </Routes>
  </Router>;
}

export default App;
