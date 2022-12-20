# Beer Delivery Robot Webapp

This repository contains a client-facing webapp for controlling a beer delivery robot. Currently, support is limited to [bwbots-beer-caddy](https://github.com/Woz4tetra/bwbots-beer-caddy).

## Installation

On the robot, ensure that ROS Noetic is installed and the [rosbridge-server](http://wiki.ros.org/rosbridge_server) package is installed.

On the client, after cloning the repository, ensure that node is installed. This [tutorial](https://heynode.com/tutorial/install-nodejs-locally-nvm/) is a good way to do this using nvm.

In the project directory, you can run:

    $ git clone https://github.com/ogunasekara/beer-delivery-robot-webapp.git
    $ cd beer-delivery-robot-webapp
    $ npm install

## Local Debugging

To start a local deploy of the application, simply run:

    $ npm start

The first run will take some time, but afterwards it will be faster. This starts up a local server on [http://localhost:3000](http://localhost:3000) with the application.

## Publishing to Github Pages

To deploy to github-pages, run:

    $ npm run deploy

This builds the app for production to the `build` folder and publishes to github pages.