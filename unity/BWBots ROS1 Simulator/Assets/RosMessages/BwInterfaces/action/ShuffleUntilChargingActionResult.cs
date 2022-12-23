using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class ShuffleUntilChargingActionResult : ActionResult<ShuffleUntilChargingResult>
    {
        public const string k_RosMessageName = "bw_interfaces/ShuffleUntilChargingActionResult";
        public override string RosMessageName => k_RosMessageName;


        public ShuffleUntilChargingActionResult() : base()
        {
            this.result = new ShuffleUntilChargingResult();
        }

        public ShuffleUntilChargingActionResult(HeaderMsg header, GoalStatusMsg status, ShuffleUntilChargingResult result) : base(header, status)
        {
            this.result = result;
        }
        public static ShuffleUntilChargingActionResult Deserialize(MessageDeserializer deserializer) => new ShuffleUntilChargingActionResult(deserializer);

        ShuffleUntilChargingActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = ShuffleUntilChargingResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
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
