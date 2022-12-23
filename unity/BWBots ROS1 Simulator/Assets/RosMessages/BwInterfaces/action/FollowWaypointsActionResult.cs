using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.BwInterfaces
{
    public class FollowWaypointsActionResult : ActionResult<FollowWaypointsResult>
    {
        public const string k_RosMessageName = "bw_interfaces/FollowWaypointsActionResult";
        public override string RosMessageName => k_RosMessageName;


        public FollowWaypointsActionResult() : base()
        {
            this.result = new FollowWaypointsResult();
        }

        public FollowWaypointsActionResult(HeaderMsg header, GoalStatusMsg status, FollowWaypointsResult result) : base(header, status)
        {
            this.result = result;
        }
        public static FollowWaypointsActionResult Deserialize(MessageDeserializer deserializer) => new FollowWaypointsActionResult(deserializer);

        FollowWaypointsActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = FollowWaypointsResult.Deserialize(deserializer);
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
