//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class SetRobotStateFeedback : Message
    {
        public const string k_RosMessageName = "bw_interfaces/SetRobotState";
        public override string RosMessageName => k_RosMessageName;


        public SetRobotStateFeedback()
        {
        }
        public static SetRobotStateFeedback Deserialize(MessageDeserializer deserializer) => new SetRobotStateFeedback(deserializer);

        private SetRobotStateFeedback(MessageDeserializer deserializer)
        {
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
        }

        public override string ToString()
        {
            return "SetRobotStateFeedback: ";
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Feedback);
        }
    }
}
