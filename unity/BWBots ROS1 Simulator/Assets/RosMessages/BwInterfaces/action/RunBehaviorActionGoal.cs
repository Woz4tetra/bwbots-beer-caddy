using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class RunBehaviorActionGoal : ActionGoal<RunBehaviorGoal>
    {
        public const string k_RosMessageName = "bw_interfaces/RunBehaviorActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public RunBehaviorActionGoal() : base()
        {
            this.goal = new RunBehaviorGoal();
        }

        public RunBehaviorActionGoal(HeaderMsg header, GoalIDMsg goal_id, RunBehaviorGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static RunBehaviorActionGoal Deserialize(MessageDeserializer deserializer) => new RunBehaviorActionGoal(deserializer);

        RunBehaviorActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = RunBehaviorGoal.Deserialize(deserializer);
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
