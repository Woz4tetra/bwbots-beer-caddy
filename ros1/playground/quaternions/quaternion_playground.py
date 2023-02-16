import math
import tf_conversions
from tf.transformations import quaternion_multiply

def euler_from_quaternion(x, y, z, w):
    quat = (x, y, z, w)
    print("     x       y       z       w")
    print("%0.4f  %0.4f  %0.4f  %0.4f" % tuple(quat))
    angles = tf_conversions.transformations.euler_from_quaternion(quat)
    print("roll x  pitch y   yaw z")
    print("%0.4f  %0.4f  %0.4f" % tuple(map(math.degrees, angles)))

def quaternion_from_euler(roll, pitch, yaw):
    print("roll x  pitch y   yaw z")
    print("%0.4f  %0.4f  %0.4f" % (roll, pitch, yaw))
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    quat = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
    print("     x       y       z       w")
    print("%0.4f  %0.4f  %0.4f  %0.4f" % tuple(quat))
    return quat

# euler_from_quaternion(0.5000,  0.5000,  -0.5000,  -0.5000)
# euler_from_quaternion(-0.0145, -0.0063, 0.2245, 0.9743)
# euler_from_quaternion(-0.0147, 0.0058, -0.5302, 0.8477)
# euler_from_quaternion(0.0, 0.0, -0.2270181849675262, 0.9738905193573095)
# quaternion_from_euler(-90.0 + 3.0, 8.0, 90.0)
# quaternion_from_euler(0.0, -42.5124239, 180.0)
quaternion_from_euler(0.0, 0.0, 90.0)

# print(quaternion_multiply((0.5, 0.5, -0.5, 0.5), quaternion_from_euler(180.0, 0.0, 0.0)))