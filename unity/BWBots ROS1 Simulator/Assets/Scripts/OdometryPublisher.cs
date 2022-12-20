using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class OdometryPublisher : MonoBehaviour
{
    [SerializeField] private ArticulationWheelController chassis;
    [SerializeField] private double publishDelay;
    [SerializeField] private string topic;
    
    private ROSConnection _ros;
    private double _prevPublishTime;
    
    // Start is called before the first frame update
    public void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<OdometryMsg>(topic);
        _prevPublishTime = Time.realtimeSinceStartup;
        Debug.Log($"Odometry topic registered: {topic}");
    }

    // Update is called once per frame
    public void Update()
    {
        double now = Time.realtimeSinceStartup;
        if (now - _prevPublishTime > publishDelay)
        {
            _ros.Publish(topic, chassis.GetOdometryMessage());
            _prevPublishTime = now;
        }
    }
}
