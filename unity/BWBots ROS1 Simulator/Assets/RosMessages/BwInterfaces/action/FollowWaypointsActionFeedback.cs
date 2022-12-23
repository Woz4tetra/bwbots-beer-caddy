using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class FollowWaypointsActionFeedback : ActionFeedback<FollowWaypointsFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/FollowWaypointsActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public FollowWaypointsActionFeedback() : base()
        {
            this.feedback = new FollowWaypointsFeedback();
        }

        public FollowWaypointsActionFeedback(HeaderMsg header, GoalStatusMsg status, FollowWaypointsFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static FollowWaypointsActionFeedback Deserialize(MessageDeserializer deserializer) => new FollowWaypointsActionFeedback(deserializer);

        FollowWaypointsActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = FollowWaypointsFeedback.Deserialize(deserializer);
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
