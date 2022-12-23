using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.BwInterfaces
{
    public class GoToPoseAction : Action<GoToPoseActionGoal, GoToPoseActionResult, GoToPoseActionFeedback, GoToPoseGoal, GoToPoseResult, GoToPoseFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/GoToPoseAction";
        public override string RosMessageName => k_RosMessageName;


        public GoToPoseAction() : base()
        {
            this.action_goal = new GoToPoseActionGoal();
            this.action_result = new GoToPoseActionResult();
            this.action_feedback = new GoToPoseActionFeedback();
        }

        public static GoToPoseAction Deserialize(MessageDeserializer deserializer) => new GoToPoseAction(deserializer);

        GoToPoseAction(MessageDeserializer deserializer)
        {
            this.action_goal = GoToPoseActionGoal.Deserialize(deserializer);
            this.action_result = GoToPoseActionResult.Deserialize(deserializer);
            this.action_feedback = GoToPoseActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
