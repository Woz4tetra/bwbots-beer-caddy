using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

class DrinkDispenser : MonoBehaviour
{
    [SerializeField] private string dispenserType;
    [SerializeField] private GameObject drink;
    [SerializeField] private string dispenseTopic = "/bw/simulated_dispense";
    [SerializeField] private string dispenserName;
    private Vector3 spawnPoint;
    private ROSConnection _ros;

    void Start() {
        Debug.Log($"Initializing dispenser type {dispenserType}");
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.Subscribe<StringMsg>(dispenseTopic, dispenseCallback);
        if (dispenserType.Equals("A0")) {
            spawnPoint = new Vector3(0.0f, 0.29f, 0.0f);
        }
        else {
            spawnPoint = new Vector3(0.0f, 1.0f, 0.0f);
        }
    }

    private void dispenseCallback(StringMsg msg) {
        if (dispenserName.Equals(msg.data)) {
            Debug.Log($"Dispensing from {dispenserName}");
            spawnDrink(drink);
        }
    }

    private void spawnDrink(GameObject drink) {
        Vector3 worldSpawnPoint = transform.TransformPoint(spawnPoint);
        Instantiate(drink, worldSpawnPoint, drink.transform.rotation);
    }
}
