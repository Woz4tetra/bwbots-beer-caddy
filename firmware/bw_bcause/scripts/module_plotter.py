import math
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow


class ModulePlotter:
    def __init__(self, positions, x_window, y_window, plot_delay=0.01, length_multipler=1.0, min_length=0.01):
        self.plot_delay = plot_delay
        self.fig = None
        self.ax = None

        self.x_window = x_window
        self.y_window = y_window

        self.positions = positions
        self.arrows = [None for _ in range(len(self.positions))]
        self.textboxes = [None for _ in range(len(self.positions))]
        self.length_multipler = length_multipler
        self.min_length = min_length

        self.init()
        
        for index, position in enumerate(self.positions):
            self.draw_textbox(position[0], position[1], str(index))

    def init(self):
        self.fig = plt.figure()

        self.ax = self.fig.add_subplot(111)

        self.set_window()

        plt.tight_layout()
        plt.ion()
        plt.legend()
        self.fig.show()
    
    def get_heading_arrow(self, x0, y0, x1, y1, color):
        arrow = Arrow(x0, y0, x1, y1, color=color, width=0.1) 
        return self.ax.add_patch(arrow)
    
    def draw_textbox(self, x, y, text):
        self.ax.text(x, y, text, fontsize=14, verticalalignment='top')

    def set_window(self):
        self.ax.set_xlim(-self.x_window / 2, self.x_window / 2)
        self.ax.set_ylim(-self.y_window / 2, self.y_window / 2)

    def set_module_arrow(self, module_index, speed, azimuth):
        arrow_length = self.length_multipler * speed + math.copysign(self.min_length, speed)
        x0 = self.positions[module_index][0]
        y0 = self.positions[module_index][1]
        x1 = arrow_length * math.cos(azimuth)
        y1 = arrow_length * math.sin(azimuth)
        arrow = self.arrows[module_index]
        if arrow is not None:
            arrow.remove()
        self.arrows[module_index] = self.get_heading_arrow(x0, y0, x1, y1, "b")

    def clear(self):
        # plt.cla()
        self.ax.clear()
        self.set_window()

    def pause(self):
        self.fig.canvas.start_event_loop(self.plot_delay)
        self.fig.canvas.flush_events()
        # self.fig.canvas.draw()
        # plt.pause(self.plot_delay)

    def stop(self):
        plt.ioff()
        plt.show()