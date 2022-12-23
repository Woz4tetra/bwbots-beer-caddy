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
    public class FindTagGoal : Message
    {
        public const string k_RosMessageName = "bw_interfaces/FindTag";
        public override string RosMessageName => k_RosMessageName;

        public int[] tag_id;
        public string reference_frame_id;
        public DurationMsg timeout;

        public FindTagGoal()
        {
            this.tag_id = new int[0];
            this.reference_frame_id = "";
            this.timeout = new DurationMsg();
        }

        public FindTagGoal(int[] tag_id, string reference_frame_id, DurationMsg timeout)
        {
            this.tag_id = tag_id;
            this.reference_frame_id = reference_frame_id;
            this.timeout = timeout;
        }

        public static FindTagGoal Deserialize(MessageDeserializer deserializer) => new FindTagGoal(deserializer);

        private FindTagGoal(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.tag_id, sizeof(int), deserializer.ReadLength());
            deserializer.Read(out this.reference_frame_id);
            this.timeout = DurationMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.tag_id);
            serializer.Write(this.tag_id);
            serializer.Write(this.reference_frame_id);
            serializer.Write(this.timeout);
        }

        public override string ToString()
        {
            return "FindTagGoal: " +
            "\ntag_id: " + System.String.Join(", ", tag_id.ToList()) +
            "\nreference_frame_id: " + reference_frame_id.ToString() +
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
