using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class ShuffleUntilChargingActionFeedback : ActionFeedback<ShuffleUntilChargingFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/ShuffleUntilChargingActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public ShuffleUntilChargingActionFeedback() : base()
        {
            this.feedback = new ShuffleUntilChargingFeedback();
        }

        public ShuffleUntilChargingActionFeedback(HeaderMsg header, GoalStatusMsg status, ShuffleUntilChargingFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static ShuffleUntilChargingActionFeedback Deserialize(MessageDeserializer deserializer) => new ShuffleUntilChargingActionFeedback(deserializer);

        ShuffleUntilChargingActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = ShuffleUntilChargingFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
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
