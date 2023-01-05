//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class GetWaypointsRequest : Message
    {
        public const string k_RosMessageName = "bw_interfaces/GetWaypoints";
        public override string RosMessageName => k_RosMessageName;

        public string[] names;

        public GetWaypointsRequest()
        {
            this.names = new string[0];
        }

        public GetWaypointsRequest(string[] names)
        {
            this.names = names;
        }

        public static GetWaypointsRequest Deserialize(MessageDeserializer deserializer) => new GetWaypointsRequest(deserializer);

        private GetWaypointsRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.names, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.names);
            serializer.Write(this.names);
        }

        public override string ToString()
        {
            return "GetWaypointsRequest: " +
            "\nnames: " + System.String.Join(", ", names.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}