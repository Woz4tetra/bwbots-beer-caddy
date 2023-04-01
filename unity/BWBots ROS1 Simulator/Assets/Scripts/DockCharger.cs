using RosMessageTypes.BwInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

class DockCharger : MonoBehaviour
{
    private ChargeStateMsg chargeState;
    [SerializeField] private double publishDelay;
    [SerializeField] private string topic;
    [SerializeField] private GameObject robotPins;
    private double _prevPublishTime;
    private Collider dock_collision;
    private Collider robot_collision;

    private ROSConnection _ros;

    void Start() {
        chargeState = new ChargeStateMsg();
        chargeState.battery_voltage = 12.0f;
        
        _ros = ROSConnection.GetOrCreateInstance();
        if (topic.Length > 0) {
            _ros.RegisterPublisher<ChargeStateMsg>(topic);
            _prevPublishTime = Time.realtimeSinceStartup;
            Debug.Log($"Charge topic registered: {topic}");
        }
        dock_collision = GetComponent<Collider>();
        robot_collision = robotPins.GetComponent<Collider>();
    }

    void FixedUpdate() {
        if (publishDelay == 0.0) {
            return;
        }
        double now = Time.realtimeSinceStartup;
        if (now - _prevPublishTime > publishDelay)
        {
            if (dock_collision.bounds.Intersects(robot_collision.bounds)) {
                chargeState.battery_voltage = 11.6f;
                chargeState.charge_current = 0.5f;
            }
            else {
                chargeState.battery_voltage = 11.6f;
                chargeState.charge_current = 0.0f;
            }
            _ros.Publish(topic, chargeState);
            _prevPublishTime = now;
        }
    }
}