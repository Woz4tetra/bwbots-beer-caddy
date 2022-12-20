using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class LaserScanPublisher : MonoBehaviour
{
    [SerializeField] private LaserScanSensor laserScanSensor;
    [SerializeField] private double scanPublishDelay;
    [SerializeField] private string topic;
    
    private ROSConnection _ros;
    private double _prevScanTime;
    
    // Start is called before the first frame update
    public void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<LaserScanMsg>(topic);
        _prevScanTime = Time.realtimeSinceStartup;
        Debug.Log($"Laser topic registered: {topic}");
    }

    // Update is called once per frame
    public void Update()
    {
        double now = Time.realtimeSinceStartup;
        if (now - _prevScanTime > scanPublishDelay)
        {
            _ros.Publish(topic, laserScanSensor.GetLaserScanMsg());
            _prevScanTime = now;
        }
    }
}
