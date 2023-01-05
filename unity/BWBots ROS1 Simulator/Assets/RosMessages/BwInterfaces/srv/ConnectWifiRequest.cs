//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class ConnectWifiRequest : Message
    {
        public const string k_RosMessageName = "bw_interfaces/ConnectWifi";
        public override string RosMessageName => k_RosMessageName;

        public string ssid;
        public string password;

        public ConnectWifiRequest()
        {
            this.ssid = "";
            this.password = "";
        }

        public ConnectWifiRequest(string ssid, string password)
        {
            this.ssid = ssid;
            this.password = password;
        }

        public static ConnectWifiRequest Deserialize(MessageDeserializer deserializer) => new ConnectWifiRequest(deserializer);

        private ConnectWifiRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.ssid);
            deserializer.Read(out this.password);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.ssid);
            serializer.Write(this.password);
        }

        public override string ToString()
        {
            return "ConnectWifiRequest: " +
            "\nssid: " + ssid.ToString() +
            "\npassword: " + password.ToString();
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