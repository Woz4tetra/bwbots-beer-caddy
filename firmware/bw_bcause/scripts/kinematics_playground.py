import numpy as np

class Kinematics:
    def __init__(self):
        self.width = 0.115  # meters
        self.length = 0.160  # meters
        self.armature = 0.037  # meters

        self.locations = [
            [self.width / 2.0, self.length / 2.0],
            [-self.width / 2.0, self.length / 2.0],
            [-self.width / 2.0, -self.length / 2.0],
            [self.width / 2.0, -self.length / 2.0],
        ]

        inverse_kinematics = []

        for row in self.locations:
            x, y = row
            inverse_kinematics.append([1.0, 0.0, -y])
            inverse_kinematics.append([0.0, 1.0, x])
        self.inverse_kinematics = np.array(inverse_kinematics)
        self.forward_kinematics = np.linalg.pinv(self.inverse_kinematics)
        print(self.inverse_kinematics)
        print(self.forward_kinematics)

    def to_module_states(self, vx, vy, vt):
        chassis_state = np.array([vx, vy, vt])
        center_module_vectors = self.inverse_kinematics.dot(chassis_state)
        next_ik = []
        for index in range(0, len(self.locations) * 2, 2):
            vx = center_module_vectors[index]
            vy = center_module_vectors[index + 1]
            angle = np.arctan2(vy, vx)
            print(vx, vy, angle)
            x = self.armature * np.cos(angle)
            y = self.armature * np.sin(angle)
            next_ik.append([1.0, 0.0, -(y + self.locations[index // 2][0])])
            next_ik.append([0.0, 1.0, (x + self.locations[index // 2][1])])
        
        next_ik = np.array(next_ik)
        print(next_ik)
        module_vectors = next_ik.dot(chassis_state)
        
        return module_vectors

def main():
    kinematics = Kinematics()
    module_states = kinematics.to_module_states(0.0, 0.0, 3.0)
    
    for index in range(0, len(module_states), 2):
        print(
            module_states[index],
            module_states[index + 1]
        )

if __name__ == '__main__':
    main()

