using RosMessageTypes.BwInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class ChassisRosInterface : MonoBehaviour
{
    [SerializeField] private BwbotsSimulatedChassis chassis;
    [SerializeField] private double publishDelay = 0.03;
    [SerializeField] private string odometryTopic = "/bw/odom";
    [SerializeField] private string jointTopicFormat = "/bw/joint/base_link_to_module_{0}_joint";
    [SerializeField] private string moduleTopicFormat = "/bw/module/{0}";
    [SerializeField] private string cmdVelTopic = "/bw/cmd_vel";
    [SerializeField] private string setMotorEnableTopic = "/bw/set_motors_enabled";
    [SerializeField] private string getMotorEnableTopic = "/bw/are_motors_enabled";
    [SerializeField] private string groundTruthTopic = "/bw/ground_truth";
    [SerializeField] private string groundTruthFrame = "map";
    
    private ROSConnection _ros;
    private double _prevPublishTime;
    private uint groundTruthMessageCount = 0;
    
    // Start is called before the first frame update
    public void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<OdometryMsg>(odometryTopic);
        _prevPublishTime = Time.realtimeSinceStartup;
        for (int index = 0; index < chassis.getNumModules(); index++) {
            _ros.RegisterPublisher<Float64Msg>(getJointTopic(index));
            _ros.RegisterPublisher<BwDriveModuleMsg>(getModuleTopic(index));
        }
        _ros.RegisterPublisher<BoolMsg>(getMotorEnableTopic);
        _ros.RegisterPublisher<PoseStampedMsg>(groundTruthTopic);
        _ros.Subscribe<BoolMsg>(setMotorEnableTopic, setMotorEnableCallback);

        _ros.Subscribe<TwistMsg>(cmdVelTopic, cmdVelCallback);
    }

    // Update is called once per frame
    public void Update()
    {
        double now = Time.realtimeSinceStartup;
        if (now - _prevPublishTime > publishDelay)
        {
            _ros.Publish(odometryTopic, chassis.GetOdometryMessage());
            for (int index = 0; index < chassis.getNumModules(); index++) {
                BwDriveModuleMsg moduleMsg = chassis.getModuleMessage(index);
                Float64Msg jointMsg = new Float64Msg {data=moduleMsg.azimuth_position};
                _ros.Publish(getJointTopic(index), jointMsg);
                _ros.Publish(getModuleTopic(index), moduleMsg);
            }
            _ros.Publish(getMotorEnableTopic, new BoolMsg {
                data = chassis.getMotorEnable()
            });

            PoseStampedMsg groundTruthMsg = new PoseStampedMsg {
                header = {
                    seq = groundTruthMessageCount,
                    stamp = RosUtil.GetTimeMsg(),
                    frame_id = groundTruthFrame
                },
                pose = chassis.GetGroundTruthPose()
            };
            _ros.Publish(groundTruthTopic, groundTruthMsg);
            groundTruthMessageCount++;

            _prevPublishTime = now;
        }
    }

    private void setMotorEnableCallback(BoolMsg msg) {
        chassis.setMotorEnable(msg.data);
    }

    private void cmdVelCallback(TwistMsg twist) {
        chassis.setTwistCommand(twist, 100, 0.5f);
    }

    private string getJointTopic(int index) {
        return string.Format(jointTopicFormat, index + 1);
    }

    private string getModuleTopic(int index) {
        return string.Format(moduleTopicFormat, index + 1);
    }
}
