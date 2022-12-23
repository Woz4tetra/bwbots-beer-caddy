//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class SaveTFRequest : Message
    {
        public const string k_RosMessageName = "bw_interfaces/SaveTF";
        public override string RosMessageName => k_RosMessageName;

        public string name;
        public string parent_frame;
        public string child_frame;

        public SaveTFRequest()
        {
            this.name = "";
            this.parent_frame = "";
            this.child_frame = "";
        }

        public SaveTFRequest(string name, string parent_frame, string child_frame)
        {
            this.name = name;
            this.parent_frame = parent_frame;
            this.child_frame = child_frame;
        }

        public static SaveTFRequest Deserialize(MessageDeserializer deserializer) => new SaveTFRequest(deserializer);

        private SaveTFRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.name);
            deserializer.Read(out this.parent_frame);
            deserializer.Read(out this.child_frame);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.name);
            serializer.Write(this.parent_frame);
            serializer.Write(this.child_frame);
        }

        public override string ToString()
        {
            return "SaveTFRequest: " +
            "\nname: " + name.ToString() +
            "\nparent_frame: " + parent_frame.ToString() +
            "\nchild_frame: " + child_frame.ToString();
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
