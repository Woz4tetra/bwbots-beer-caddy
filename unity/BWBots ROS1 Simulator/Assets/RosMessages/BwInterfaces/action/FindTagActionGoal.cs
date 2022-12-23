using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class FindTagActionGoal : ActionGoal<FindTagGoal>
    {
        public const string k_RosMessageName = "bw_interfaces/FindTagActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public FindTagActionGoal() : base()
        {
            this.goal = new FindTagGoal();
        }

        public FindTagActionGoal(HeaderMsg header, GoalIDMsg goal_id, FindTagGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static FindTagActionGoal Deserialize(MessageDeserializer deserializer) => new FindTagActionGoal(deserializer);

        FindTagActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = FindTagGoal.Deserialize(deserializer);
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
