using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.BwInterfaces
{
    public class HasDrinkAction : Action<HasDrinkActionGoal, HasDrinkActionResult, HasDrinkActionFeedback, HasDrinkGoal, HasDrinkResult, HasDrinkFeedback>
    {
        public const string k_RosMessageName = "bw_interfaces/HasDrinkAction";
        public override string RosMessageName => k_RosMessageName;


        public HasDrinkAction() : base()
        {
            this.action_goal = new HasDrinkActionGoal();
            this.action_result = new HasDrinkActionResult();
            this.action_feedback = new HasDrinkActionFeedback();
        }

        public static HasDrinkAction Deserialize(MessageDeserializer deserializer) => new HasDrinkAction(deserializer);

        HasDrinkAction(MessageDeserializer deserializer)
        {
            this.action_goal = HasDrinkActionGoal.Deserialize(deserializer);
            this.action_result = HasDrinkActionResult.Deserialize(deserializer);
            this.action_feedback = HasDrinkActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
