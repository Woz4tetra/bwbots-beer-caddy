using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class SetRobotStateActionGoal : ActionGoal<SetRobotStateGoal>
    {
        public const string k_RosMessageName = "bw_interfaces/SetRobotStateActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public SetRobotStateActionGoal() : base()
        {
            this.goal = new SetRobotStateGoal();
        }

        public SetRobotStateActionGoal(HeaderMsg header, GoalIDMsg goal_id, SetRobotStateGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static SetRobotStateActionGoal Deserialize(MessageDeserializer deserializer) => new SetRobotStateActionGoal(deserializer);

        SetRobotStateActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = SetRobotStateGoal.Deserialize(deserializer);
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
