//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class SetRobotStateResult : Message
    {
        public const string k_RosMessageName = "bw_interfaces/SetRobotState";
        public override string RosMessageName => k_RosMessageName;

        public bool success;

        public SetRobotStateResult()
        {
            this.success = false;
        }

        public SetRobotStateResult(bool success)
        {
            this.success = success;
        }

        public static SetRobotStateResult Deserialize(MessageDeserializer deserializer) => new SetRobotStateResult(deserializer);

        private SetRobotStateResult(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.success);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.success);
        }

        public override string ToString()
        {
            return "SetRobotStateResult: " +
            "\nsuccess: " + success.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Result);
        }
    }
}