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
    public class SetRobotStateGoal : Message
    {
        public const string k_RosMessageName = "bw_interfaces/SetRobotState";
        public override string RosMessageName => k_RosMessageName;

        public bool enabled;
        public DurationMsg timeout;

        public SetRobotStateGoal()
        {
            this.enabled = false;
            this.timeout = new DurationMsg();
        }

        public SetRobotStateGoal(bool enabled, DurationMsg timeout)
        {
            this.enabled = enabled;
            this.timeout = timeout;
        }

        public static SetRobotStateGoal Deserialize(MessageDeserializer deserializer) => new SetRobotStateGoal(deserializer);

        private SetRobotStateGoal(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.enabled);
            this.timeout = DurationMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.enabled);
            serializer.Write(this.timeout);
        }

        public override string ToString()
        {
            return "SetRobotStateGoal: " +
            "\nenabled: " + enabled.ToString() +
            "\ntimeout: " + timeout.ToString();
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