#!/usr/bin/env python3
import re
import os
import rospy
import subprocess
from dataclasses import dataclass
from typing import Optional, Tuple

from bw_interfaces.srv import ConnectWifi, ConnectWifiResponse
from bw_interfaces.srv import ListWifi, ListWifiResponse
from bw_interfaces.msg import WifiNetwork


@dataclass
class Connection:
    name: str
    uuid: str
    conn_type: str
    device: str


class BwHotspot:
    def __init__(self) -> None:
        rospy.init_node("bw_hotspot")
        
        self.connect_wifi_srv = rospy.Service("connect_wifi", ConnectWifi, self.connect_callback)
        self.list_wifi_srv = rospy.Service("list_wifi", ListWifi, self.list_callback)
        
        rospy.loginfo("bw_hotspot is ready")

    def connect_callback(self, req: ConnectWifi):
        success, msg = self.connect(req.ssid, req.password)
        return ConnectWifiResponse(success, msg)

    def list_callback(self, req: ListWifi):
        self.rescan_ssids(req.scan_time)
        result, msg = self.list_ssids()
        if result is None:
            success = False
            networks = []
        else:
            success = True
            networks = list(result)
            
        return ListWifiResponse(networks, success, msg)

    def connect(self, name, password) -> Tuple[bool, str]:
        stored = any([connection.name == name for connection in self.list_connections()])
        if stored:
            return self.connect_to_existing_network(name)
        else:
            return self.connect_to_new_network(name, password)

    def connect_to_existing_network(self, name: str) -> Tuple[bool, str]:
        rospy.loginfo(f"Connecting to {name}")
        command = ["sudo", "nmcli", "con", "up", str(name)]
        proc = subprocess.Popen(command, stdout=subprocess.PIPE)
        output, error = proc.communicate()
        result = self.combine_output(output, error)
        if "successfully activated" in result:
            rospy.loginfo("Connection successful!")
            return True, result
        else:
            rospy.loginfo("Connection failed!")
            return False, result

    def disconnect_from_connection(self, name: str) -> Tuple[bool, str]:
        rospy.loginfo(f"Disconnecting from {name}")
        command = ["sudo", "nmcli", "con", "down", str(name)]
        proc = subprocess.Popen(command, stdout=subprocess.PIPE)
        output, error = proc.communicate()
        result = self.combine_output(output, error)
        if "successfully deactivated" in result:
            return True, result
        else:
            return False, result

    def connect_to_new_network(self, ssid: str, password: str) -> Tuple[bool, str]:
        rospy.loginfo(f"Connecting to a new network named {ssid}")
        command = ["sudo", "nmcli", "dev", "wifi", "connect", str(ssid), "password", str(password)]
        proc = subprocess.Popen(command, stdout=subprocess.PIPE)
        output, error = proc.communicate()
        result = self.combine_output(output, error)
        if "successfully activated" in result:
            return True, result
        else:
            return False, result

    def list_connections(self) -> Tuple[Connection]:
        rospy.loginfo("Listing existing connections")
        regex = r"^([\S\s]+)\s{2}(\S+)\s{2}(\S+)\s+(\S+)\s*"

        proc = subprocess.Popen(["nmcli", "con"], stdout=subprocess.PIPE)
        output, error = proc.communicate()
        connections = []
        for line in output.decode().splitlines()[1:]:
            match = re.search(regex, line)
            if not match:
                continue
            name = match.group(1).strip()
            uuid = match.group(2).strip()
            conn_type = match.group(3).strip()
            device = match.group(4).strip()
            connections.append(Connection(name, uuid, conn_type, device))
        rospy.loginfo(f"Found {len(connections)} connections")
        return tuple(connections)

    def combine_output(self, output: bytes, error: bytes) -> str:
        if output is None:
            output = ""
        else:
            output = output.decode()
        if error is None:
            error = ""
        else:
            error = error.decode()
        result = output + "\n"
        result += error
        return result

    def rescan_ssids(self, scan_time: rospy.Duration) -> Tuple[bool, str]:
        rospy.loginfo(f"Scanning wifi networks. Delaying for {scan_time.to_sec()} seconds")
        proc = subprocess.Popen("sudo nmcli dev wifi rescan".split(" "), stdout=subprocess.PIPE)
        output, error = proc.communicate()
        result = self.combine_output(output, error)
        if len(result.strip()) > 0:
            return False, result
        else:
            rospy.sleep(scan_time)
            return True, ""

    def list_ssids(self) -> Tuple[Optional[Tuple[WifiNetwork]], str]:
        rospy.loginfo("Listing wifi networks")
        proc = subprocess.Popen("sudo nmcli dev wifi list".split(" "), stdout=subprocess.PIPE)
        output, error = proc.communicate()
        if error is not None and len(error) > 0:
            msg = f"Encountered error while listing networks: {error.decode()}"
            rospy.logwarn(msg)
            return None, msg
        if output is None:
            output = ""
        pattern = r"\s{2}"
        networks = []
        for line in output.decode().splitlines()[1:]:
            match = list(filter(lambda x: len(x.strip()) > 0, re.split(pattern, line)))
            if match[0] != "*":
                match.insert(0, "")
            while len(match) != 8:
                if len(match) < 8:
                    msg = f"Invalid result found while scanning for networks: '{line}'"
                    rospy.logwarn(msg)
                    return None, msg
                match[1] += " " + match.pop(2)
            in_use = match[0] == "*"
            ssid = match[1]
            mode = match[2]
            try:
                channel = int(match[3])
            except ValueError:
                channel = -1
            rate = -1
            try:
                rate_match = re.search("(\d+)", match[5])
                if rate_match:
                    rate = int(rate_match.group(1))
            except ValueError:
                pass
            try:
                signal = int(match[5])
            except ValueError:
                signal = -1
            bars = match[6]
            security = match[7]
            networks.append(WifiNetwork(
                in_use,
                ssid,
                mode,
                channel,
                rate,
                signal,
                bars,
                security,
            ))
        rospy.loginfo(f"Found {len(networks)} networks. Top result: {networks[0]}")
        return tuple(networks), ""

    def run(self):
        rospy.spin()


def main():
    node = BwHotspot()
    node.run()


if __name__ == '__main__':
    main()
