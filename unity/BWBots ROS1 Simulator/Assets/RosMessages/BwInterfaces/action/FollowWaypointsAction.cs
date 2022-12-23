using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.BwInterfaces
{
    public class FollowWaypointsAction : Action<FollowWaypointsActionGoal, FollowWaypointsActionResult, FollowWaypointsActionFeedback, FollowWaypointsGoal, FollowWaypointsResult, FollowWaypointsFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/FollowWaypointsAction";
        public override string RosMessageName => k_RosMessageName;


        public FollowWaypointsAction() : base()
        {
            this.action_goal = new FollowWaypointsActionGoal();
            this.action_result = new FollowWaypointsActionResult();
            this.action_feedback = new FollowWaypointsActionFeedback();
        }

        public static FollowWaypointsAction Deserialize(MessageDeserializer deserializer) => new FollowWaypointsAction(deserializer);

        FollowWaypointsAction(MessageDeserializer deserializer)
        {
            this.action_goal = FollowWaypointsActionGoal.Deserialize(deserializer);
            this.action_result = FollowWaypointsActionResult.Deserialize(deserializer);
            this.action_feedback = FollowWaypointsActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
