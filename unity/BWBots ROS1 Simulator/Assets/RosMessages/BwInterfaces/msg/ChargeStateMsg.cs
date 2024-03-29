//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class ChargeStateMsg : Message
    {
        public const string k_RosMessageName = "bw_interfaces/ChargeState";
        public override string RosMessageName => k_RosMessageName;

        public float battery_voltage;
        public float charge_current;

        public ChargeStateMsg()
        {
            this.battery_voltage = 0.0f;
            this.charge_current = 0.0f;
        }

        public ChargeStateMsg(float battery_voltage, float charge_current)
        {
            this.battery_voltage = battery_voltage;
            this.charge_current = charge_current;
        }

        public static ChargeStateMsg Deserialize(MessageDeserializer deserializer) => new ChargeStateMsg(deserializer);

        private ChargeStateMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.battery_voltage);
            deserializer.Read(out this.charge_current);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.battery_voltage);
            serializer.Write(this.charge_current);
        }

        public override string ToString()
        {
            return "ChargeStateMsg: " +
            "\nbattery_voltage: " + battery_voltage.ToString() +
            "\ncharge_current: " + charge_current.ToString();
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
