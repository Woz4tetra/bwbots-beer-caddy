using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class HasDrinkActionGoal : ActionGoal<HasDrinkGoal>
    {
        public const string k_RosMessageName = "bw_interfaces/HasDrinkActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public HasDrinkActionGoal() : base()
        {
            this.goal = new HasDrinkGoal();
        }

        public HasDrinkActionGoal(HeaderMsg header, GoalIDMsg goal_id, HasDrinkGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static HasDrinkActionGoal Deserialize(MessageDeserializer deserializer) => new HasDrinkActionGoal(deserializer);

        HasDrinkActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = HasDrinkGoal.Deserialize(deserializer);
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
