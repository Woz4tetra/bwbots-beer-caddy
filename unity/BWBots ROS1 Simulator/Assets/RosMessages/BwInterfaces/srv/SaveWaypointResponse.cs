//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class SaveWaypointResponse : Message
    {
        public const string k_RosMessageName = "bw_interfaces/SaveWaypoint";
        public override string RosMessageName => k_RosMessageName;

        public bool success;

        public SaveWaypointResponse()
        {
            this.success = false;
        }

        public SaveWaypointResponse(bool success)
        {
            this.success = success;
        }

        public static SaveWaypointResponse Deserialize(MessageDeserializer deserializer) => new SaveWaypointResponse(deserializer);

        private SaveWaypointResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.success);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.success);
        }

        public override string ToString()
        {
            return "SaveWaypointResponse: " +
            "\nsuccess: " + success.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}