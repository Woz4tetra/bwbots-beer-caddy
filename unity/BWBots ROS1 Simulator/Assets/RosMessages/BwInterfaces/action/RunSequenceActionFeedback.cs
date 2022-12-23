using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class RunSequenceActionFeedback : ActionFeedback<RunSequenceFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/RunSequenceActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public RunSequenceActionFeedback() : base()
        {
            this.feedback = new RunSequenceFeedback();
        }

        public RunSequenceActionFeedback(HeaderMsg header, GoalStatusMsg status, RunSequenceFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static RunSequenceActionFeedback Deserialize(MessageDeserializer deserializer) => new RunSequenceActionFeedback(deserializer);

        RunSequenceActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = RunSequenceFeedback.Deserialize(deserializer);
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
