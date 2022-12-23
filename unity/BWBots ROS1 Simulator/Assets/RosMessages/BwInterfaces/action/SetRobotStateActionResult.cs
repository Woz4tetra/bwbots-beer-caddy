using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class SetRobotStateActionResult : ActionResult<SetRobotStateResult>
    {
        public const string k_RosMessageName = "bw_interfaces/SetRobotStateActionResult";
        public override string RosMessageName => k_RosMessageName;


        public SetRobotStateActionResult() : base()
        {
            this.result = new SetRobotStateResult();
        }

        public SetRobotStateActionResult(HeaderMsg header, GoalStatusMsg status, SetRobotStateResult result) : base(header, status)
        {
            this.result = result;
        }
        public static SetRobotStateActionResult Deserialize(MessageDeserializer deserializer) => new SetRobotStateActionResult(deserializer);

        SetRobotStateActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = SetRobotStateResult.Deserialize(deserializer);
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
