import React from 'react';
import { Link } from 'react-router-dom';
import { constructPath } from '../App';

const menuItems = ["Connect", "Deliver", "Waypoints", "State"];
const routes = ["connect", "deliver", "waypoints", "state"];

export default function NavBar() {
    return <div style={{display: "flex"}}>
        {menuItems.map((menuItem, index) => 
            <div style={{flexGrow: 1, textAlign: "center", padding: "20px"}}>
                <Link to={constructPath(routes[index])}>
                    {menuItem}
                </Link>
            </div>
        )}
    </div>
}