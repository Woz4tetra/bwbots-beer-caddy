using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class RunSequenceActionGoal : ActionGoal<RunSequenceGoal>
    {
        public const string k_RosMessageName = "bw_interfaces/RunSequenceActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public RunSequenceActionGoal() : base()
        {
            this.goal = new RunSequenceGoal();
        }

        public RunSequenceActionGoal(HeaderMsg header, GoalIDMsg goal_id, RunSequenceGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static RunSequenceActionGoal Deserialize(MessageDeserializer deserializer) => new RunSequenceActionGoal(deserializer);

        RunSequenceActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = RunSequenceGoal.Deserialize(deserializer);
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
