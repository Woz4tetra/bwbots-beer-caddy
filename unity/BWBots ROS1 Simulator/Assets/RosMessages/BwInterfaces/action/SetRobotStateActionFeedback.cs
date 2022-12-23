using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class SetRobotStateActionFeedback : ActionFeedback<SetRobotStateFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/SetRobotStateActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public SetRobotStateActionFeedback() : base()
        {
            this.feedback = new SetRobotStateFeedback();
        }

        public SetRobotStateActionFeedback(HeaderMsg header, GoalStatusMsg status, SetRobotStateFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static SetRobotStateActionFeedback Deserialize(MessageDeserializer deserializer) => new SetRobotStateActionFeedback(deserializer);

        SetRobotStateActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = SetRobotStateFeedback.Deserialize(deserializer);
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
