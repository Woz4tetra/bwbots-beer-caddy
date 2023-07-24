import pyperclip
import numpy as np
from typing import Tuple
import tqdm
import matplotlib
from matplotlib.cm import get_cmap
from matplotlib import pyplot as plt

from nav_msgs.msg import Odometry

from bw_interfaces.msg import GoToPoseActionGoal
from move_base_msgs.msg import MoveBaseActionGoal

from bw_tools.robot_state import Pose2d, Velocity
from bw_tools.rosbag_to_file.utils import enumerate_bag, Options, get_bag_length

matplotlib.use("TkAgg")

def odometry_msg_to_pose2d(msg: Odometry) -> Tuple[Pose2d, Velocity]:
    return Pose2d.from_ros_pose(msg.pose.pose), Velocity.from_ros_twist(msg.twist.twist)


def init_data_containers():
    odometries = {
        "/bw/odom": [],
        "/bw/odom/filtered": [],
        "/drive_calibration/tag_odom": [],
        "/drive_calibration/amcl_odom": [],
    }
    go_to_pose_events = []
    return odometries, go_to_pose_events

def load(path):
    options = Options(path, "")
    bag_generator = enumerate_bag(options)
    bag_length = get_bag_length(options)
    
    odometries, go_to_pose_events = init_data_containers()
    go_to_pose_events.append({})
    for topic, msg, timestamp in bag_generator:
        if topic in odometries:
            pose2d, velocity = odometry_msg_to_pose2d(msg)
            odometries[topic].append([msg.header.stamp.to_sec(), pose2d, velocity])
        elif topic == "/bw/go_to_pose/goal":
            go_to_pose_events[-1]["start"] = timestamp.to_sec()
            go_to_pose_events[-1]["goal"] = msg.goal
        elif topic == "/move_base/goal":
            go_to_pose_events[-1]["start"] = timestamp.to_sec()
            go_to_pose_events[-1]["goal"] = msg.goal.target_pose
        elif topic in ("/bw/go_to_pose/result", "/move_base/result"):
            if "start" not in go_to_pose_events[-1]:
                print("Goal finish event doesn't have a corresponding start event! Ignoring.")
                continue
            go_to_pose_events[-1]["stop"] = timestamp.to_sec()
            go_to_pose_events.append({})

    if len(go_to_pose_events[-1]) == 0:
        go_to_pose_events.pop(-1)
    return odometries, go_to_pose_events

def load_all(*paths):
    odometries, go_to_pose_events = init_data_containers()
    for path in paths:
        next_odometries, next_go_to_pose_events = load(path)
        for topic in odometries:
            odometries[topic].extend(next_odometries[topic])
        go_to_pose_events.extend(next_go_to_pose_events)
    return odometries, go_to_pose_events

def plot_all_runs(odometries, go_to_pose_events):
    # topics = list(odometries.keys())
    # colors = get_cmap("twilight").colors
    # color_map = {}
    # for index, topic in enumerate(topics):
    #     color_index = int(index * len(colors) / len(topics))
    #     color_map[topic] = colors[color_index]
    
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set_aspect('equal', 'box')
    
    for topic, odometry_data in odometries.items():    
        start_pose = odometry_data[0][1]
        poses = []
        velocities = []
        for row in odometry_data:
            poses.append(row[1].relative_to(start_pose).to_list())
            velocities.append(row[2].to_list())
        poses = np.array(poses)
        velocities = np.array(velocities)

        # speed = np.linalg.norm(velocities[:, 0:2], axis=1)
        # arrow_length = np.clip(speed, 0.0001, None)
        arrow_length = 0.01
        x = poses[:, 0]
        y = poses[:, 1]
        theta = poses[:, 2]
        dx = arrow_length * np.cos(theta)
        dy = arrow_length * np.sin(theta)
        line = ax.plot(x, y, '-', label=topic)[0]
        ax.quiver(x, y, dx, dy, angles='uv', scale_units='xy', scale=1.0, color=line.get_color(), label=topic)
        
        xmin, xmax = ax.get_xlim()
        ymin, ymax = ax.get_ylim()
        values = xmin, xmax, ymin, ymax
        max_value = np.max(np.abs(values))
        ax.set_xlim(-max_value, max_value)
        ax.set_ylim(-max_value, max_value)
    ax.legend()

def nearest_index(data, value) -> int:
    return np.argmin(np.abs(data - value))

def is_velocity_stopped(velocity: Velocity):
    return velocity.magnitude() < 0.01 and abs(velocity.theta) < 0.01

def get_start_stop_poses(odometry_data, go_to_pose_events):
    start_stops = []
    for index, event in enumerate(go_to_pose_events):
        timestamps = np.array([row[0] for row in odometry_data])
        start_index = nearest_index(timestamps, event["start"])
        start_pose = odometry_data[start_index][1]
        start_velocity = odometry_data[start_index][2]

        stop_index = nearest_index(timestamps, event["stop"])
        stop_pose = odometry_data[stop_index][1]
        stop_velocity = odometry_data[stop_index][2]
        
        if not is_velocity_stopped(start_velocity):
            print(f"Event #{index} at {timestamps[start_index]} did not start with zero velocity.")
        if not is_velocity_stopped(stop_velocity):
            print(f"Event #{index} at {timestamps[start_index]} did not stop with zero velocity.")
            continue

        start_stops.append((start_pose, stop_pose))
    return start_stops

def tabulate_final_error(odometries, go_to_pose_events):
    errors = {topic: [] for topic in odometries}
    for topic in odometries:
        start_stops = get_start_stop_poses(odometries[topic], go_to_pose_events)
        for index, (start_pose, stop_pose) in enumerate(start_stops):
            event = go_to_pose_events[index]
            frame_id = event["goal"].goal.header.frame_id
            goal_pose = Pose2d.from_ros_pose(event["goal"].goal.pose)
            if frame_id == "odom":
                error = goal_pose.relative_to(stop_pose)
            else:
                relative_pose = stop_pose.relative_to(start_pose)
                error = goal_pose.relative_to(relative_pose)
            errors[topic].append([event["start"]] + error.to_list())

    for topic in errors:
        data = np.array(errors[topic])
        distances = np.linalg.norm(data[:, 1:3], axis=1)
        distance_error = np.mean(distances)
        distance_stddev = np.std(distances)
        x_error = np.mean(data[:, 1])
        y_error = np.mean(data[:, 2])
        theta_error = np.mean(data[:, 3])
        print(f"{topic} means: x={x_error}, y={y_error}, theta={theta_error}, distance={distance_error}, distance stddev={distance_stddev}")

    spreadsheet_text = ""
    for topic in errors:
        for data in errors[topic]:
            row = [topic] + errors[topic]
            row_str = "\t".join([str(x) for x in row])
            spreadsheet_text += row_str + "\n"
    pyperclip.copy(spreadsheet_text)

def main():
    odometries, go_to_pose_events = load_all(
        "/root/bags/drive_calibration_2023-02-16-05-52-29.bag",
    )
    tabulate_final_error(odometries, go_to_pose_events)
    plot_all_runs(odometries, go_to_pose_events)
    plt.show()


main()
