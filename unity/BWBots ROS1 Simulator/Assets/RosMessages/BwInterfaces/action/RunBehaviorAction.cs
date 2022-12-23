using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.BwInterfaces
{
    public class RunBehaviorAction : Action<RunBehaviorActionGoal, RunBehaviorActionResult, RunBehaviorActionFeedback, RunBehaviorGoal, RunBehaviorResult, RunBehaviorFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/RunBehaviorAction";
        public override string RosMessageName => k_RosMessageName;


        public RunBehaviorAction() : base()
        {
            this.action_goal = new RunBehaviorActionGoal();
            this.action_result = new RunBehaviorActionResult();
            this.action_feedback = new RunBehaviorActionFeedback();
        }

        public static RunBehaviorAction Deserialize(MessageDeserializer deserializer) => new RunBehaviorAction(deserializer);

        RunBehaviorAction(MessageDeserializer deserializer)
        {
            this.action_goal = RunBehaviorActionGoal.Deserialize(deserializer);
            this.action_result = RunBehaviorActionResult.Deserialize(deserializer);
            this.action_feedback = RunBehaviorActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
