using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.BwInterfaces
{
    public class RunSequenceAction : Action<RunSequenceActionGoal, RunSequenceActionResult, RunSequenceActionFeedback, RunSequenceGoal, RunSequenceResult, RunSequenceFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/RunSequenceAction";
        public override string RosMessageName => k_RosMessageName;


        public RunSequenceAction() : base()
        {
            this.action_goal = new RunSequenceActionGoal();
            this.action_result = new RunSequenceActionResult();
            this.action_feedback = new RunSequenceActionFeedback();
        }

        public static RunSequenceAction Deserialize(MessageDeserializer deserializer) => new RunSequenceAction(deserializer);

        RunSequenceAction(MessageDeserializer deserializer)
        {
            this.action_goal = RunSequenceActionGoal.Deserialize(deserializer);
            this.action_result = RunSequenceActionResult.Deserialize(deserializer);
            this.action_feedback = RunSequenceActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
