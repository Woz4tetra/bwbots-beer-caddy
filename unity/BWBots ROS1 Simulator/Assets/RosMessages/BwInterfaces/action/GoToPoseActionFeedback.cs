using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class GoToPoseActionFeedback : ActionFeedback<GoToPoseFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/GoToPoseActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public GoToPoseActionFeedback() : base()
        {
            this.feedback = new GoToPoseFeedback();
        }

        public GoToPoseActionFeedback(HeaderMsg header, GoalStatusMsg status, GoToPoseFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static GoToPoseActionFeedback Deserialize(MessageDeserializer deserializer) => new GoToPoseActionFeedback(deserializer);

        GoToPoseActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = GoToPoseFeedback.Deserialize(deserializer);
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
