
import math
import numpy as np
from module_plotter import ModulePlotter

class Kinematics:
    def __init__(self):
        self.width = 0.115  # meters
        self.length = 0.160  # meters
        self.armature = 0.037  # meters

        self.locations = [
            [self.length / 2.0, self.width / 2.0],  # module 3, channel 2, front left
            [-self.length / 2.0, self.width / 2.0],  # module 1, channel 0, back left
            [-self.length / 2.0, -self.width / 2.0],  # module 2, channel 1, back right
            [self.length / 2.0, -self.width / 2.0],  # module 4, channel 3, front right
        ]
    
    def compute_module_state(self, x, y, vx, vy, vt, dt):
        theta_mag = vt * dt
        if theta_mag == 0.0:
            module_vx = vx
            module_vy = vy
        else:
            v_mag = math.sqrt(vx * vx + vy * vy)
            d_mag = v_mag * dt
            radius_of_curvature = d_mag / math.tan(theta_mag)
            # print(radius_of_curvature)
            if radius_of_curvature == 0.0:
                module_vx = vx + vt * -y
                module_vy = vy + vt * x
            elif abs(radius_of_curvature) < 0.1:
                module_vx = vt * -y
                module_vy = vt * x
            else:
                module_angle = math.atan2(x, radius_of_curvature + y)
                module_radc = x / math.sin(module_angle) - self.armature

                module_vx = v_mag * module_radc / radius_of_curvature * math.cos(module_angle)
                module_vy = v_mag * module_radc / radius_of_curvature * math.sin(module_angle)
        azimuth = math.atan2(module_vy, module_vx)
        wheel_velocity = math.sqrt(module_vx * module_vx + module_vy * module_vy)
        return azimuth, wheel_velocity

    def to_module_states(self, vx, vy, vt, dt):
        left = self.compute_module_state(self.length / 2.0, self.width / 2.0, vx, vy, vt, dt)
        right = self.compute_module_state(self.length / 2.0, -self.width / 2.0, vx, vy, vt, dt)

        module_states = [
            (left[0], left[1]),  # channel 2
            (-left[0], left[1]),  # channel 0
            (-right[0], right[1]),  # channel 1
            (right[0], right[1]),  # channel 3
        ]

        return module_states

def main():
    kinematics = Kinematics()
    module_states = kinematics.to_module_states(0.0, 0.0, 2.0, 1.0 / 60.0)
    module_plotter = ModulePlotter(kinematics.locations, 2.0, 2.0)
    
    for module_num in range(len(module_states)):
        azimuth, wheel_velocity = module_states[module_num]
        print(azimuth, wheel_velocity)
        module_plotter.set_module_arrow(module_num, wheel_velocity, azimuth)
        module_plotter.pause()
    module_plotter.stop()


if __name__ == '__main__':
    main()

