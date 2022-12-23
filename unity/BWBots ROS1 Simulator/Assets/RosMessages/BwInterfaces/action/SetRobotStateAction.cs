using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.BwInterfaces
{
    public class SetRobotStateAction : Action<SetRobotStateActionGoal, SetRobotStateActionResult, SetRobotStateActionFeedback, SetRobotStateGoal, SetRobotStateResult, SetRobotStateFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/SetRobotStateAction";
        public override string RosMessageName => k_RosMessageName;


        public SetRobotStateAction() : base()
        {
            this.action_goal = new SetRobotStateActionGoal();
            this.action_result = new SetRobotStateActionResult();
            this.action_feedback = new SetRobotStateActionFeedback();
        }

        public static SetRobotStateAction Deserialize(MessageDeserializer deserializer) => new SetRobotStateAction(deserializer);

        SetRobotStateAction(MessageDeserializer deserializer)
        {
            this.action_goal = SetRobotStateActionGoal.Deserialize(deserializer);
            this.action_result = SetRobotStateActionResult.Deserialize(deserializer);
            this.action_feedback = SetRobotStateActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
