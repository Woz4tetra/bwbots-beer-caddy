using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BwInterfaces;
using System.Collections.Generic;

class LoadCellSensor : MonoBehaviour
{
    private LoadCellMsg loadCellMsg;
    [SerializeField] private double publishDelay;
    [SerializeField] private string topic;
    private double _prevPublishTime;
    private uint messageCount;
    List<GameObject> carryingObjects;

    private ROSConnection _ros;

    void Start() {
        loadCellMsg = new LoadCellMsg();
        carryingObjects = new List<GameObject>();
        
        _ros = ROSConnection.GetOrCreateInstance();
        if (topic.Length > 0) {
            _ros.RegisterPublisher<LoadCellMsg>(topic);
            _prevPublishTime = Time.realtimeSinceStartup;
            Debug.Log($"Load cell topic registered: {topic}");
        }
    }

    void FixedUpdate() {
        if (publishDelay == 0.0) {
            return;
        }
        Rigidbody body;
        double totalMass = 0.0;
        foreach (GameObject obj in carryingObjects) {
            if (obj.TryGetComponent<Rigidbody>(out body)) {
                totalMass += body.mass;
            }
        }
        double now = Time.realtimeSinceStartup;
        if (now - _prevPublishTime > publishDelay)
        {
            _ros.Publish(topic, loadCellMsg);
            _prevPublishTime = now;
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        Rigidbody body;
        if (collision.gameObject.TryGetComponent<Rigidbody>(out body)) {
            if (!carryingObjects.Contains(collision.gameObject)) {
                Debug.Log($"collided with {collision.gameObject.name}. It weighs {body.mass} kg");
                carryingObjects.Add(collision.gameObject);
            }
        }
    }

    void OnCollisionExit(Collision collision)
    {
        if (carryingObjects.Contains(collision.gameObject)) {
            carryingObjects.Remove(collision.gameObject);
            Debug.Log($"no longer collided with {collision.gameObject.name}");
        }
    }
}