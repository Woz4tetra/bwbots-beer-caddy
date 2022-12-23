using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class HasDrinkActionResult : ActionResult<HasDrinkResult>
    {
        public const string k_RosMessageName = "bw_interfaces/HasDrinkActionResult";
        public override string RosMessageName => k_RosMessageName;


        public HasDrinkActionResult() : base()
        {
            this.result = new HasDrinkResult();
        }

        public HasDrinkActionResult(HeaderMsg header, GoalStatusMsg status, HasDrinkResult result) : base(header, status)
        {
            this.result = result;
        }
        public static HasDrinkActionResult Deserialize(MessageDeserializer deserializer) => new HasDrinkActionResult(deserializer);

        HasDrinkActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = HasDrinkResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
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
