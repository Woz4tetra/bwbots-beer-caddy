using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class FindTagActionFeedback : ActionFeedback<FindTagFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/FindTagActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public FindTagActionFeedback() : base()
        {
            this.feedback = new FindTagFeedback();
        }

        public FindTagActionFeedback(HeaderMsg header, GoalStatusMsg status, FindTagFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static FindTagActionFeedback Deserialize(MessageDeserializer deserializer) => new FindTagActionFeedback(deserializer);

        FindTagActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = FindTagFeedback.Deserialize(deserializer);
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
