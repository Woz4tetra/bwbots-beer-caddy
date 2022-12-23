using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.BwInterfaces
{
    public class ShuffleUntilChargingAction : Action<ShuffleUntilChargingActionGoal, ShuffleUntilChargingActionResult, ShuffleUntilChargingActionFeedback, ShuffleUntilChargingGoal, ShuffleUntilChargingResult, ShuffleUntilChargingFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/ShuffleUntilChargingAction";
        public override string RosMessageName => k_RosMessageName;


        public ShuffleUntilChargingAction() : base()
        {
            this.action_goal = new ShuffleUntilChargingActionGoal();
            this.action_result = new ShuffleUntilChargingActionResult();
            this.action_feedback = new ShuffleUntilChargingActionFeedback();
        }

        public static ShuffleUntilChargingAction Deserialize(MessageDeserializer deserializer) => new ShuffleUntilChargingAction(deserializer);

        ShuffleUntilChargingAction(MessageDeserializer deserializer)
        {
            this.action_goal = ShuffleUntilChargingActionGoal.Deserialize(deserializer);
            this.action_result = ShuffleUntilChargingActionResult.Deserialize(deserializer);
            this.action_feedback = ShuffleUntilChargingActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
