import math
from matplotlib import pyplot as plt
import numpy as np
from bag_loader import get_states
import scipy.optimize

def hysteresis_meas_to_set(x, a, b):
    return a * np.tan(x * b)



def main():
    data = get_states("bags/module_2022-10-07-13-08-53.json")  # no min command, no PID, Kp = 0.0, Ki = 0.0, Kd = 0.0, Kf = 255.0 / 1.0, speed K = 0.9

    module_num = 2

    start_time = 0.0
    stop_time = 12.9

    data_start_time = data[module_num]["measurement"][0]["time"]
    data_stop_time = data[module_num]["measurement"][-1]["time"]

    if stop_time is None:
        stop_time = data_stop_time
    if start_time is None:
        start_time = data_start_time

    measurement_times = []
    velocity_measurements = []
    for msg in data[module_num]["measurement"]:
        timestamp = msg["time"] - data_start_time
        if start_time <= timestamp <= stop_time:
            measurement_times.append(timestamp)
            velocity_measurements.append(msg["wheel_velocity"])

    setpoint_times = []
    velocity_setpoints = []
    for msg in data[module_num]["setpoint"]:
        timestamp = msg["time"] - data_start_time
        if start_time <= timestamp <= stop_time:
            setpoint_times.append(timestamp)
            velocity_setpoints.append(msg["wheel_velocity"])

    velocity_setpoints = np.array(velocity_setpoints)
    velocity_measurements = np.array(velocity_measurements)

    interp_velocity_setpoints = np.interp(measurement_times, setpoint_times, velocity_setpoints)
    interp_velocity_measurements = np.interp(setpoint_times, measurement_times, velocity_measurements)

    meas_to_set_popt, meas_to_set_pcov = scipy.optimize.curve_fit(hysteresis_meas_to_set, interp_velocity_measurements, velocity_setpoints)
    print(tuple(meas_to_set_popt.tolist()))

    plt.figure(1)
    plt.plot(measurement_times, velocity_measurements, label="measured (m/s)")
    # plt.plot(setpoint_times, velocity_setpoints, label="setpoint (m/s)")
    plt.plot(measurement_times, interp_velocity_setpoints, label="setpoint (m/s)")
    plt.plot(setpoint_times, hysteresis_meas_to_set(interp_velocity_measurements, *meas_to_set_popt), label="fitted meas to set (m/s)")
    plt.plot(setpoint_times, hysteresis_meas_to_set(velocity_setpoints, *meas_to_set_popt), label="set to fitted set (m/s)")
    plt.legend()

    plt.figure(2)
    plt.plot(interp_velocity_setpoints, velocity_measurements, '.')
    plt.plot(hysteresis_meas_to_set(interp_velocity_measurements, *meas_to_set_popt), interp_velocity_measurements, label="fitted meas to set")
    plt.xlabel("setpoint")
    plt.ylabel("measured")
    plt.legend()

    plt.figure(3)
    setpoint_range = np.linspace(-1.05, 1.05, 100)
    plt.plot(setpoint_range, hysteresis_meas_to_set(setpoint_range, *meas_to_set_popt), label="meas_to_set")
    plt.legend()

    plt.show()

main()
