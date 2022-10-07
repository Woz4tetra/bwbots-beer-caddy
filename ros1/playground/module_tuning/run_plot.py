import math
from matplotlib import pyplot as plt
import numpy as np
from bag_loader import get_states
import scipy.optimize

def hysteresis_set_to_meas(x, a, b):
    return a * np.arctan(x * b)
    # sign = np.copysign(1.0, x)
    # return sign * a * (np.abs(x) * b) ** (1/3)

def hysteresis_meas_to_set(x, a, b):
    # return a * (x * b) ** 3
    return a * np.tan(x * b)



def main():
    # data = get_states("bags/module_2022-10-07-12-28-55.json")  # joystick deadzone, semi-tuned PID, Kp = 4.0, Ki = 0.0, Kd = 0.001, Kf = 255.0 / 1.0, speed K = 0.9
    # data = get_states("bags/module_2022-10-07-12-49-07.json")  # joystick deadzone, no PID, Kp = 0.0, Ki = 0.0, Kd = 0.0, Kf = 255.0 / 1.0, speed K = 0.9
    # data = get_states("bags/module_2022-10-07-12-56-10.json")  # no PID, Kp = 0.0, Ki = 0.0, Kd = 0.0, Kf = 255.0 / 1.0, speed K = 0.9
    # data = get_states("bags/module_2022-10-07-13-03-58.json")  # no min command, raw instead of m/s, no PID, Kp = 0.0, Ki = 0.0, Kd = 0.0, Kf = 255.0 / 1.0, speed K = 0.9
    data = get_states("bags/module_2022-10-07-13-08-53.json")  # no min command, no PID, Kp = 0.0, Ki = 0.0, Kd = 0.0, Kf = 255.0 / 1.0, speed K = 0.9
    # data = get_states("bags/module_2022-10-07-15-07-06.json")  # tan(x) adjustment, no PID, Kp = 0.0, Ki = 0.0, Kd = 0.0, Kf = 255.0 / 1.0, speed K = 0.9


    module_num = 2

    start_time = data[module_num]["measurement"][0]["time"]

    measurement_times = []
    velocity_measurements = []
    for msg in data[module_num]["measurement"]:
        measurement_times.append(msg["time"] - start_time)
        velocity_measurements.append(msg["wheel_velocity"])

    setpoint_times = []
    velocity_setpoints = []
    for msg in data[module_num]["setpoint"]:
        setpoint_times.append(msg["time"] - start_time)
        velocity_setpoints.append(msg["wheel_velocity"])

    velocity_setpoints = np.array(velocity_setpoints)
    velocity_measurements = np.array(velocity_measurements)
    # velocity_measurements *= np.max(velocity_setpoints) / 255.0  # for bags/module_2022-10-07-13-03-58.json

    interp_velocity_setpoints = np.interp(measurement_times, setpoint_times, velocity_setpoints)
    interp_velocity_measurements = np.interp(setpoint_times, measurement_times, velocity_measurements)

    set_to_meas_popt, set_to_meas_pcov = scipy.optimize.curve_fit(hysteresis_set_to_meas, interp_velocity_setpoints, velocity_measurements)
    print(tuple(set_to_meas_popt.tolist()))
    meas_to_set_popt, meas_to_set_pcov = scipy.optimize.curve_fit(hysteresis_meas_to_set, interp_velocity_measurements, velocity_setpoints)
    print(tuple(meas_to_set_popt.tolist()))
    # set_to_meas_popt = (0.7890882730565366, 4.309578498174602)
    # meas_to_set_popt = (0.12527425549711227, 1.418961080687716)



    plt.figure(1)
    plt.plot(measurement_times, velocity_measurements, label="measured (m/s)")
    # plt.plot(setpoint_times, velocity_setpoints, label="setpoint (m/s)")
    plt.plot(measurement_times, interp_velocity_setpoints, label="setpoint (m/s)")
    plt.plot(measurement_times, hysteresis_set_to_meas(interp_velocity_setpoints, *set_to_meas_popt), label="fitted set to meas (m/s)")
    plt.plot(setpoint_times, hysteresis_meas_to_set(interp_velocity_measurements, *meas_to_set_popt), label="fitted meas to set (m/s)")
    plt.legend()

    plt.figure(2)
    plt.plot(interp_velocity_setpoints, velocity_measurements, '.')
    plt.plot(interp_velocity_setpoints, hysteresis_set_to_meas(interp_velocity_setpoints, *set_to_meas_popt), label="fitted set to meas")
    plt.plot(hysteresis_meas_to_set(interp_velocity_measurements, *meas_to_set_popt), interp_velocity_measurements, label="fitted meas to set")
    plt.xlabel("setpoint")
    plt.ylabel("measured")
    plt.legend()

    plt.figure(3)
    setpoint_range = np.linspace(-1.05, 1.05, 100)
    plt.plot(setpoint_range, hysteresis_set_to_meas(setpoint_range, *set_to_meas_popt), label="set_to_meas")
    plt.plot(setpoint_range, hysteresis_meas_to_set(setpoint_range, *meas_to_set_popt), label="meas_to_set")
    plt.legend()

    plt.show()

main()
