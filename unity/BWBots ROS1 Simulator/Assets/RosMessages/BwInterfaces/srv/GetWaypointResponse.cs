//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class GetWaypointResponse : Message
    {
        public const string k_RosMessageName = "bw_interfaces/GetWaypoint";
        public override string RosMessageName => k_RosMessageName;

        public WaypointMsg waypoint;
        public bool success;

        public GetWaypointResponse()
        {
            this.waypoint = new WaypointMsg();
            this.success = false;
        }

        public GetWaypointResponse(WaypointMsg waypoint, bool success)
        {
            this.waypoint = waypoint;
            this.success = success;
        }

        public static GetWaypointResponse Deserialize(MessageDeserializer deserializer) => new GetWaypointResponse(deserializer);

        private GetWaypointResponse(MessageDeserializer deserializer)
        {
            this.waypoint = WaypointMsg.Deserialize(deserializer);
            deserializer.Read(out this.success);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.waypoint);
            serializer.Write(this.success);
        }

        public override string ToString()
        {
            return "GetWaypointResponse: " +
            "\nwaypoint: " + waypoint.ToString() +
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