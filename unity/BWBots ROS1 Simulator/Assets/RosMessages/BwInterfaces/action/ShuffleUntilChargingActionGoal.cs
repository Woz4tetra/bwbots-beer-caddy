using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class ShuffleUntilChargingActionGoal : ActionGoal<ShuffleUntilChargingGoal>
    {
        public const string k_RosMessageName = "bw_interfaces/ShuffleUntilChargingActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public ShuffleUntilChargingActionGoal() : base()
        {
            this.goal = new ShuffleUntilChargingGoal();
        }

        public ShuffleUntilChargingActionGoal(HeaderMsg header, GoalIDMsg goal_id, ShuffleUntilChargingGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static ShuffleUntilChargingActionGoal Deserialize(MessageDeserializer deserializer) => new ShuffleUntilChargingActionGoal(deserializer);

        ShuffleUntilChargingActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = ShuffleUntilChargingGoal.Deserialize(deserializer);
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
