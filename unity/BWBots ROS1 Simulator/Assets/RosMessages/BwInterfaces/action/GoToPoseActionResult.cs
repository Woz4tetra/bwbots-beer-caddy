using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class GoToPoseActionResult : ActionResult<GoToPoseResult>
    {
        public const string k_RosMessageName = "bw_interfaces/GoToPoseActionResult";
        public override string RosMessageName => k_RosMessageName;


        public GoToPoseActionResult() : base()
        {
            this.result = new GoToPoseResult();
        }

        public GoToPoseActionResult(HeaderMsg header, GoalStatusMsg status, GoToPoseResult result) : base(header, status)
        {
            this.result = result;
        }
        public static GoToPoseActionResult Deserialize(MessageDeserializer deserializer) => new GoToPoseActionResult(deserializer);

        GoToPoseActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = GoToPoseResult.Deserialize(deserializer);
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
