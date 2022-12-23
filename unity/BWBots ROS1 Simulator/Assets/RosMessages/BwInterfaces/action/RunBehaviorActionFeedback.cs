using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class RunBehaviorActionFeedback : ActionFeedback<RunBehaviorFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/RunBehaviorActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public RunBehaviorActionFeedback() : base()
        {
            this.feedback = new RunBehaviorFeedback();
        }

        public RunBehaviorActionFeedback(HeaderMsg header, GoalStatusMsg status, RunBehaviorFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static RunBehaviorActionFeedback Deserialize(MessageDeserializer deserializer) => new RunBehaviorActionFeedback(deserializer);

        RunBehaviorActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = RunBehaviorFeedback.Deserialize(deserializer);
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
