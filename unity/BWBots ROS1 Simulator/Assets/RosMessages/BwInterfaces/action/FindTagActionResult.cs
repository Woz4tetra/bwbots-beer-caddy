using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class FindTagActionResult : ActionResult<FindTagResult>
    {
        public const string k_RosMessageName = "bw_interfaces/FindTagActionResult";
        public override string RosMessageName => k_RosMessageName;


        public FindTagActionResult() : base()
        {
            this.result = new FindTagResult();
        }

        public FindTagActionResult(HeaderMsg header, GoalStatusMsg status, FindTagResult result) : base(header, status)
        {
            this.result = result;
        }
        public static FindTagActionResult Deserialize(MessageDeserializer deserializer) => new FindTagActionResult(deserializer);

        FindTagActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = FindTagResult.Deserialize(deserializer);
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
