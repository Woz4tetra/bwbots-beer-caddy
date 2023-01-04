//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.ZedInterfaces
{
    [Serializable]
    public class BoundingBox3DMsg : Message
    {
        public const string k_RosMessageName = "zed_interfaces/BoundingBox3D";
        public override string RosMessageName => k_RosMessageName;

        //       1 ------- 2
        //      /.        /|
        //     0 ------- 3 |
        //     | .       | |
        //     | 5.......| 6
        //     |.        |/
        //     4 ------- 7
        public Keypoint3DMsg[] corners;

        public BoundingBox3DMsg()
        {
            this.corners = new Keypoint3DMsg[8];
        }

        public BoundingBox3DMsg(Keypoint3DMsg[] corners)
        {
            this.corners = corners;
        }

        public static BoundingBox3DMsg Deserialize(MessageDeserializer deserializer) => new BoundingBox3DMsg(deserializer);

        private BoundingBox3DMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.corners, Keypoint3DMsg.Deserialize, 8);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.corners);
        }

        public override string ToString()
        {
            return "BoundingBox3DMsg: " +
            "\ncorners: " + System.String.Join(", ", corners.ToList());
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
