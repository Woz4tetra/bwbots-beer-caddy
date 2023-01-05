//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class CalibrateScaleResponse : Message
    {
        public const string k_RosMessageName = "bw_interfaces/CalibrateScale";
        public override string RosMessageName => k_RosMessageName;

        public float calibration_value;

        public CalibrateScaleResponse()
        {
            this.calibration_value = 0.0f;
        }

        public CalibrateScaleResponse(float calibration_value)
        {
            this.calibration_value = calibration_value;
        }

        public static CalibrateScaleResponse Deserialize(MessageDeserializer deserializer) => new CalibrateScaleResponse(deserializer);

        private CalibrateScaleResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.calibration_value);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.calibration_value);
        }

        public override string ToString()
        {
            return "CalibrateScaleResponse: " +
            "\ncalibration_value: " + calibration_value.ToString();
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