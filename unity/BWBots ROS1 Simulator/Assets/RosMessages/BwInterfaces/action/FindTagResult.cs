//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class FindTagResult : Message
    {
        public const string k_RosMessageName = "bw_interfaces/FindTag";
        public override string RosMessageName => k_RosMessageName;

        public Geometry.PoseStampedMsg pose;
        public bool success;

        public FindTagResult()
        {
            this.pose = new Geometry.PoseStampedMsg();
            this.success = false;
        }

        public FindTagResult(Geometry.PoseStampedMsg pose, bool success)
        {
            this.pose = pose;
            this.success = success;
        }

        public static FindTagResult Deserialize(MessageDeserializer deserializer) => new FindTagResult(deserializer);

        private FindTagResult(MessageDeserializer deserializer)
        {
            this.pose = Geometry.PoseStampedMsg.Deserialize(deserializer);
            deserializer.Read(out this.success);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.pose);
            serializer.Write(this.success);
        }

        public override string ToString()
        {
            return "FindTagResult: " +
            "\npose: " + pose.ToString() +
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