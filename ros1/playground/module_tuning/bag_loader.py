from tj2_tools.rosbag_to_file.json_loader import iter_bag, get_key, header_to_stamp, yaw_from_quat, read_pkl


def read_states(path):
    data = [{"measurement": [], "setpoint": []} for _ in range(4)]
    print("Creating pickle from %s" % path)

    for timestamp, topic, msg in iter_bag(path):
        if topic == "/bw/module" or topic == "/bw/module_command":
            index = get_key(msg, "module_index")
            index = int(index)
            if index >= 4 or index < 0:
                continue

            row = {
                "time": timestamp,
                "wheel_velocity": get_key(msg, "wheel_velocity"),
                "wheel_position": get_key(msg, "wheel_position"),
                "azimuth": get_key(msg, "azimuth_position"),
            }
            if topic == "/bw/module":
                key = "measurement"
            elif topic == "/bw/module_command":
                key = "setpoint"
            else:
                key = ""

            data[index][key].append(row)

    return data


def get_states(path, repickle=False):
    return read_pkl(path, read_states, repickle)
