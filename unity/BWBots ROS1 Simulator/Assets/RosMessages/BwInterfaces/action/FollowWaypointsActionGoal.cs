using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class FollowWaypointsActionGoal : ActionGoal<FollowWaypointsGoal>
    {
        public const string k_RosMessageName = "bw_interfaces/FollowWaypointsActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public FollowWaypointsActionGoal() : base()
        {
            this.goal = new FollowWaypointsGoal();
        }

        public FollowWaypointsActionGoal(HeaderMsg header, GoalIDMsg goal_id, FollowWaypointsGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static FollowWaypointsActionGoal Deserialize(MessageDeserializer deserializer) => new FollowWaypointsActionGoal(deserializer);

        FollowWaypointsActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = FollowWaypointsGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
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
