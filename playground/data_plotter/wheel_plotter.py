import argparse
import time
import rospy
import threading

import matplotlib

matplotlib.use('TkAgg')
from matplotlib import pyplot as plt

from bw_interfaces.msg import BwDriveModule, BwDriveState


class DataTracker:
    def __init__(self, axis, time_window, y_min=-0.01, y_max=0.01) -> None:
        self.ax = axis
        self.time_window = time_window
        self.times = []
        self.lines = {}
        self.data = {}
        self.y_min = y_min
        self.y_max = y_max
        self.lock = threading.Lock()
    
    def add_line(self, name, color):
        line = self.ax.plot([], [], label=name, color=color)[0]
        self.lines[name] = line
        self.data[name] = []

    def add_data(self, timestamp, data: dict):
        with self.lock:
            assert data.keys() == self.data.keys()
            self.times.append(timestamp)
            for key, value in data.items():
                self.data[key].append(value)
            
                if value < self.y_min:
                    self.y_min = value
                if value > self.y_max:
                    self.y_max = value

            while timestamp - self.times[0] > self.time_window:
                self.times.pop(0)
                for key, value in data.items():
                    self.data[key].pop(0)

    def draw(self):
        with self.lock:
            if len(self.times) == 0:
                return
            for key in self.data:
                line = self.lines[key]
                line.set_xdata(self.times)
                line.set_ydata(self.data[key])
            timestamp = self.times[-1]
            self.ax.set_xlim(timestamp - self.time_window, timestamp)
            self.ax.set_ylim(self.y_min, self.y_max)


class Plotter:
    def __init__(self, plot_delay: float) -> None:
        self.plot_delay = plot_delay

        self.fig = plt.figure(1)
        plt.tight_layout()
        plt.ion()
        self.fig.show()

        self.ax1 = self.fig.add_subplot(1, 1, 1)
        self.module_data = DataTracker(self.ax1, 30.0)
        self.module_keys = ['module1', 'module2', 'module3', 'module4']
        module_color = ['red', 'blue', 'green', 'orange']
        module_setpoint_color = ['salmon', 'lightskyblue', 'lightgreen', 'bisque']
        for index, key in enumerate(self.module_keys):
            color = module_color[index]
            setpoint_color = module_setpoint_color[index]
            self.module_data.add_line(key, color)
            self.module_data.add_line(key + "_setpoint", setpoint_color)
        self.ax1.legend()
        
        self.trackers = [self.module_data]

        self.start_time = time.time()

        fig_manager = plt.get_current_fig_manager()
        fig_manager.resize(*fig_manager.window.maxsize())

    def draw_module(self, msg: BwDriveState):
        data = {}
        for index, key in enumerate(self.module_keys):
            module: BwDriveModule = msg.modules[index]
            data[key] = module.wheel_velocity
            data[key + "_setpoint"] = module.wheel_velocity_setpoint
        self.module_data.add_data(msg.header.stamp.to_sec(), data)

    def clear(self) -> None:
        plt.cla()

    def pause(self) -> None:
        for tracker in self.trackers:
            tracker.draw()
        backend = plt.rcParams['backend']
        if backend in matplotlib.rcsetup.interactive_bk:
            figManager = matplotlib._pylab_helpers.Gcf.get_active()
            if figManager is not None:
                canvas = figManager.canvas
                if canvas.figure.stale:
                    canvas.draw()
                canvas.start_event_loop(self.plot_delay)
                return


def main() -> None:
    # parser = argparse.ArgumentParser(description='wheel_plotter')
    # args = parser.parse_args()
    
    rospy.init_node("wheel_plotter")
    
    drive_sub = rospy.Subscriber("/bw/modules", BwDriveState, lambda msg: plotter.draw_module(msg))

    plotter = Plotter(0.01)

    try:
        while True:
            time.sleep(0.015)

            plotter.pause()
            if not plt.fignum_exists(1):
                break
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()


if __name__ == '__main__':
    main()
