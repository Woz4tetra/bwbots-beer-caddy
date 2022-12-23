using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.BwInterfaces
{
    public class FindTagAction : Action<FindTagActionGoal, FindTagActionResult, FindTagActionFeedback, FindTagGoal, FindTagResult, FindTagFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/FindTagAction";
        public override string RosMessageName => k_RosMessageName;


        public FindTagAction() : base()
        {
            this.action_goal = new FindTagActionGoal();
            this.action_result = new FindTagActionResult();
            this.action_feedback = new FindTagActionFeedback();
        }

        public static FindTagAction Deserialize(MessageDeserializer deserializer) => new FindTagAction(deserializer);

        FindTagAction(MessageDeserializer deserializer)
        {
            this.action_goal = FindTagActionGoal.Deserialize(deserializer);
            this.action_result = FindTagActionResult.Deserialize(deserializer);
            this.action_feedback = FindTagActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
