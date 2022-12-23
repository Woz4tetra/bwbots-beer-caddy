using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class HasDrinkActionFeedback : ActionFeedback<HasDrinkFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/HasDrinkActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public HasDrinkActionFeedback() : base()
        {
            this.feedback = new HasDrinkFeedback();
        }

        public HasDrinkActionFeedback(HeaderMsg header, GoalStatusMsg status, HasDrinkFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static HasDrinkActionFeedback Deserialize(MessageDeserializer deserializer) => new HasDrinkActionFeedback(deserializer);

        HasDrinkActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = HasDrinkFeedback.Deserialize(deserializer);
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
