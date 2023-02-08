using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

class ImuSensor : MonoBehaviour
{
    public string FrameId = "imu";

    private ImuMsg imuMsg;
    public ArticulationBody bodyMain;
    private Vector3 prevVelocity = new Vector3();

    [SerializeField] private double publishDelay;
    [SerializeField] private string topic;
    private double _prevPublishTime;
    private uint messageCount;

    private ROSConnection _ros;
    private Quaternion startOrientation;

    void Start() {
        imuMsg = new ImuMsg();
        
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<ImuMsg>(topic);
        _prevPublishTime = Time.realtimeSinceStartup;
        Debug.Log($"Imu topic registered: {topic}");

        imuMsg.header.frame_id = FrameId;

        imuMsg.orientation_covariance = new double[] {
            1e-1, 0.0, 0.0,
            0.0, 1e-1, 0.0,
            0.0, 0.0, 1e-1
        };
        imuMsg.angular_velocity_covariance = new double[] {
            1e-1, 0.0, 0.0,
            0.0, 1e-1, 0.0,
            0.0, 0.0, 1e-1
        };
        imuMsg.linear_acceleration_covariance = new double[] {
            1e-2, 0.0, 0.0,
            0.0, 1e-2, 0.0,
            0.0, 0.0, 1e-2
        };

        startOrientation = Quaternion.Inverse(bodyMain.transform.rotation);
    }

    void FixedUpdate() {
        double now = Time.realtimeSinceStartup;
        if (now - _prevPublishTime > publishDelay)
        {
            _ros.Publish(topic, imuMsg);
            _prevPublishTime = now;
        }

        imuMsg.header.stamp = RosUtil.GetTimeMsg();
        imuMsg.header.seq = messageCount;

        Vector3 velocity = transform.InverseTransformDirection(bodyMain.velocity);
        float dt = Time.fixedDeltaTime;
        Vector3 accel = new Vector3(
            (velocity.x - prevVelocity.x) / dt,
            (velocity.y - prevVelocity.y) / dt,
            (velocity.z - prevVelocity.z) / dt
        );
        prevVelocity = velocity;
        imuMsg.linear_acceleration = accel.To<FLU>();

        imuMsg.angular_velocity = -bodyMain.angularVelocity.To<FLU>();

        imuMsg.orientation = (bodyMain.transform.rotation * startOrientation).To<FLU>();
        messageCount++;
    }
}