using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BwInterfaces;
using System.Collections.Generic;

class LoadCellSensor : MonoBehaviour
{
    private LoadCellMsg loadCellMsg;
    [SerializeField] private double publishDelay;
    [SerializeField] private string topic;
    [SerializeField] private string tagName;
    private double _prevPublishTime;
    private uint messageCount;
    List<GameObject> carryingObjects;

    private ROSConnection _ros;
    private double totalMass = 0.0;

    void Start()
    {
        loadCellMsg = new LoadCellMsg();
        carryingObjects = new List<GameObject>();

        _ros = ROSConnection.GetOrCreateInstance();
        if (topic.Length > 0)
        {
            _ros.RegisterPublisher<LoadCellMsg>(topic);
            _prevPublishTime = Time.realtimeSinceStartup;
            Debug.Log($"Load cell topic registered: {topic}");
        }
    }

    void FixedUpdate()
    {
        if (publishDelay == 0.0)
        {
            return;
        }
        Rigidbody body;
        double massSum = 0.0;
        carryingObjects.RemoveAll(obj => obj == null);
        foreach (GameObject obj in carryingObjects)
        {
            if (obj.TryGetComponent<Rigidbody>(out body))
            {
                massSum += body.mass;
            }
        }
        if (massSum > totalMass)
        {
            totalMass = massSum;
        }
        else
        {
            totalMass += 0.1 * (massSum - totalMass);
        }
        double now = Time.realtimeSinceStartup;
        if (now - _prevPublishTime > publishDelay)
        {
            loadCellMsg.mass = (float)totalMass;
            _ros.Publish(topic, loadCellMsg);
            _prevPublishTime = now;
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        Rigidbody body;
        if (collision.gameObject.TryGetComponent<Rigidbody>(out body) &&
                collision.gameObject.tag.Equals(tagName) &&
                !carryingObjects.Contains(collision.gameObject))
        {
            carryingObjects.Add(collision.gameObject);
        }
    }

    void OnCollisionExit(Collision collision)
    {
        if (carryingObjects.Contains(collision.gameObject))
        {
            carryingObjects.Remove(collision.gameObject);
        }
    }
}