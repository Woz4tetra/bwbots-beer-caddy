#!/usr/bin/node

var ROSLIB = require('roslib');

const ros = new ROSLIB.Ros({
    url: 'ws://192.168.64.3:9090/'
});

ros.on('connection', () => {
    console.log('Connected to websocket server.');
});

ros.on('error', (error) => {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', () => {
    console.log('Connection to websocket server closed.');
});

var odomListener = new ROSLIB.Topic({
    ros: ros,
    name: '/tj2/odom',
    messageType: 'nav_msgs/msg/Odometry'
});

odomCallback = (message) => { console.log(message.pose.pose); };

odomListener.subscribe(odomCallback);