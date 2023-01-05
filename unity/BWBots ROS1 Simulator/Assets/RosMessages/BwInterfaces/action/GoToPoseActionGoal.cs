using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class GoToPoseActionGoal : ActionGoal<GoToPoseGoal>
    {
        public const string k_RosMessageName = "bw_interfaces/GoToPoseActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public GoToPoseActionGoal() : base()
        {
            this.goal = new GoToPoseGoal();
        }

        public GoToPoseActionGoal(HeaderMsg header, GoalIDMsg goal_id, GoToPoseGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static GoToPoseActionGoal Deserialize(MessageDeserializer deserializer) => new GoToPoseActionGoal(deserializer);

        GoToPoseActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = GoToPoseGoal.Deserialize(deserializer);
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