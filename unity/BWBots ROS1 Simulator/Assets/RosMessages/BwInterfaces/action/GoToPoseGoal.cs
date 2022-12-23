//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.BuiltinInterfaces;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class GoToPoseGoal : Message
    {
        public const string k_RosMessageName = "bw_interfaces/GoToPose";
        public override string RosMessageName => k_RosMessageName;

        public Geometry.PoseStampedMsg goal;
        public double xy_tolerance;
        public double yaw_tolerance;
        public DurationMsg timeout;
        public bool ignore_obstacles;
        public double reference_linear_speed;
        public double reference_angular_speed;
        public bool allow_reverse;
        public bool rotate_in_place_start;
        public bool rotate_while_driving;
        public bool rotate_in_place_end;
        public double linear_max_vel;
        public double linear_max_accel;
        public double linear_min_vel;
        public double linear_zero_vel;
        public double theta_max_vel;
        public double theta_max_accel;
        public double theta_min_vel;
        public double theta_zero_vel;
        public string controller_type;

        public GoToPoseGoal()
        {
            this.goal = new Geometry.PoseStampedMsg();
            this.xy_tolerance = 0.0;
            this.yaw_tolerance = 0.0;
            this.timeout = new DurationMsg();
            this.ignore_obstacles = false;
            this.reference_linear_speed = 0.0;
            this.reference_angular_speed = 0.0;
            this.allow_reverse = false;
            this.rotate_in_place_start = false;
            this.rotate_while_driving = false;
            this.rotate_in_place_end = false;
            this.linear_max_vel = 0.0;
            this.linear_max_accel = 0.0;
            this.linear_min_vel = 0.0;
            this.linear_zero_vel = 0.0;
            this.theta_max_vel = 0.0;
            this.theta_max_accel = 0.0;
            this.theta_min_vel = 0.0;
            this.theta_zero_vel = 0.0;
            this.controller_type = "";
        }

        public GoToPoseGoal(Geometry.PoseStampedMsg goal, double xy_tolerance, double yaw_tolerance, DurationMsg timeout, bool ignore_obstacles, double reference_linear_speed, double reference_angular_speed, bool allow_reverse, bool rotate_in_place_start, bool rotate_while_driving, bool rotate_in_place_end, double linear_max_vel, double linear_max_accel, double linear_min_vel, double linear_zero_vel, double theta_max_vel, double theta_max_accel, double theta_min_vel, double theta_zero_vel, string controller_type)
        {
            this.goal = goal;
            this.xy_tolerance = xy_tolerance;
            this.yaw_tolerance = yaw_tolerance;
            this.timeout = timeout;
            this.ignore_obstacles = ignore_obstacles;
            this.reference_linear_speed = reference_linear_speed;
            this.reference_angular_speed = reference_angular_speed;
            this.allow_reverse = allow_reverse;
            this.rotate_in_place_start = rotate_in_place_start;
            this.rotate_while_driving = rotate_while_driving;
            this.rotate_in_place_end = rotate_in_place_end;
            this.linear_max_vel = linear_max_vel;
            this.linear_max_accel = linear_max_accel;
            this.linear_min_vel = linear_min_vel;
            this.linear_zero_vel = linear_zero_vel;
            this.theta_max_vel = theta_max_vel;
            this.theta_max_accel = theta_max_accel;
            this.theta_min_vel = theta_min_vel;
            this.theta_zero_vel = theta_zero_vel;
            this.controller_type = controller_type;
        }

        public static GoToPoseGoal Deserialize(MessageDeserializer deserializer) => new GoToPoseGoal(deserializer);

        private GoToPoseGoal(MessageDeserializer deserializer)
        {
            this.goal = Geometry.PoseStampedMsg.Deserialize(deserializer);
            deserializer.Read(out this.xy_tolerance);
            deserializer.Read(out this.yaw_tolerance);
            this.timeout = DurationMsg.Deserialize(deserializer);
            deserializer.Read(out this.ignore_obstacles);
            deserializer.Read(out this.reference_linear_speed);
            deserializer.Read(out this.reference_angular_speed);
            deserializer.Read(out this.allow_reverse);
            deserializer.Read(out this.rotate_in_place_start);
            deserializer.Read(out this.rotate_while_driving);
            deserializer.Read(out this.rotate_in_place_end);
            deserializer.Read(out this.linear_max_vel);
            deserializer.Read(out this.linear_max_accel);
            deserializer.Read(out this.linear_min_vel);
            deserializer.Read(out this.linear_zero_vel);
            deserializer.Read(out this.theta_max_vel);
            deserializer.Read(out this.theta_max_accel);
            deserializer.Read(out this.theta_min_vel);
            deserializer.Read(out this.theta_zero_vel);
            deserializer.Read(out this.controller_type);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.goal);
            serializer.Write(this.xy_tolerance);
            serializer.Write(this.yaw_tolerance);
            serializer.Write(this.timeout);
            serializer.Write(this.ignore_obstacles);
            serializer.Write(this.reference_linear_speed);
            serializer.Write(this.reference_angular_speed);
            serializer.Write(this.allow_reverse);
            serializer.Write(this.rotate_in_place_start);
            serializer.Write(this.rotate_while_driving);
            serializer.Write(this.rotate_in_place_end);
            serializer.Write(this.linear_max_vel);
            serializer.Write(this.linear_max_accel);
            serializer.Write(this.linear_min_vel);
            serializer.Write(this.linear_zero_vel);
            serializer.Write(this.theta_max_vel);
            serializer.Write(this.theta_max_accel);
            serializer.Write(this.theta_min_vel);
            serializer.Write(this.theta_zero_vel);
            serializer.Write(this.controller_type);
        }

        public override string ToString()
        {
            return "GoToPoseGoal: " +
            "\ngoal: " + goal.ToString() +
            "\nxy_tolerance: " + xy_tolerance.ToString() +
            "\nyaw_tolerance: " + yaw_tolerance.ToString() +
            "\ntimeout: " + timeout.ToString() +
            "\nignore_obstacles: " + ignore_obstacles.ToString() +
            "\nreference_linear_speed: " + reference_linear_speed.ToString() +
            "\nreference_angular_speed: " + reference_angular_speed.ToString() +
            "\nallow_reverse: " + allow_reverse.ToString() +
            "\nrotate_in_place_start: " + rotate_in_place_start.ToString() +
            "\nrotate_while_driving: " + rotate_while_driving.ToString() +
            "\nrotate_in_place_end: " + rotate_in_place_end.ToString() +
            "\nlinear_max_vel: " + linear_max_vel.ToString() +
            "\nlinear_max_accel: " + linear_max_accel.ToString() +
            "\nlinear_min_vel: " + linear_min_vel.ToString() +
            "\nlinear_zero_vel: " + linear_zero_vel.ToString() +
            "\ntheta_max_vel: " + theta_max_vel.ToString() +
            "\ntheta_max_accel: " + theta_max_accel.ToString() +
            "\ntheta_min_vel: " + theta_min_vel.ToString() +
            "\ntheta_zero_vel: " + theta_zero_vel.ToString() +
            "\ncontroller_type: " + controller_type.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Goal);
        }
    }
}
