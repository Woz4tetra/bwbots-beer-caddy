//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class StopSequenceRequest : Message
    {
        public const string k_RosMessageName = "bw_interfaces/StopSequence";
        public override string RosMessageName => k_RosMessageName;


        public StopSequenceRequest()
        {
        }
        public static StopSequenceRequest Deserialize(MessageDeserializer deserializer) => new StopSequenceRequest(deserializer);

        private StopSequenceRequest(MessageDeserializer deserializer)
        {
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
        }

        public override string ToString()
        {
            return "StopSequenceRequest: ";
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
