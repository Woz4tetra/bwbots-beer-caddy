using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class RunBehaviorActionResult : ActionResult<RunBehaviorResult>
    {
        public const string k_RosMessageName = "bw_interfaces/RunBehaviorActionResult";
        public override string RosMessageName => k_RosMessageName;


        public RunBehaviorActionResult() : base()
        {
            this.result = new RunBehaviorResult();
        }

        public RunBehaviorActionResult(HeaderMsg header, GoalStatusMsg status, RunBehaviorResult result) : base(header, status)
        {
            this.result = result;
        }
        public static RunBehaviorActionResult Deserialize(MessageDeserializer deserializer) => new RunBehaviorActionResult(deserializer);

        RunBehaviorActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = RunBehaviorResult.Deserialize(deserializer);
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
