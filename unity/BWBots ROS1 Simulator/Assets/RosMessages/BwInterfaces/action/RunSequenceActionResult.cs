using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class RunSequenceActionResult : ActionResult<RunSequenceResult>
    {
        public const string k_RosMessageName = "bw_interfaces/RunSequenceActionResult";
        public override string RosMessageName => k_RosMessageName;


        public RunSequenceActionResult() : base()
        {
            this.result = new RunSequenceResult();
        }

        public RunSequenceActionResult(HeaderMsg header, GoalStatusMsg status, RunSequenceResult result) : base(header, status)
        {
            this.result = result;
        }
        public static RunSequenceActionResult Deserialize(MessageDeserializer deserializer) => new RunSequenceActionResult(deserializer);

        RunSequenceActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = RunSequenceResult.Deserialize(deserializer);
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
