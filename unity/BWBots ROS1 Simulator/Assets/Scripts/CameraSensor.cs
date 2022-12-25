using System.Collections.Generic;
using RosMessageTypes.ApriltagRos;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

class CameraSensor : MonoBehaviour
{
    GameObject[] tags;
    Camera cameraView;
    private ROSConnection _ros;
    [SerializeField] private double publishDelay;
    [SerializeField] private string tagTopic;
    [SerializeField] private string frameId;
    [SerializeField] private float rayCastOffset = 0.01f;
    [SerializeField] private float rayCastTolerance = 0.015f;
    [SerializeField] private bool debugRayCast = false;
    private double _prevPublishTime;
    private uint messageCount;

    void Awake()
    {
        tags = GameObject.FindGameObjectsWithTag("locator_tag");
        cameraView = GetComponent<Camera>();
        if (tagTopic.Length > 0) {
            _ros = ROSConnection.GetOrCreateInstance();
            _ros.RegisterPublisher<AprilTagDetectionArrayMsg>(tagTopic);
        }
    }

    void Update() 
    {
        double now = Time.realtimeSinceStartup;

        AprilTagDetectionArrayMsg tagArrayMsg = new AprilTagDetectionArrayMsg {
            header = {
                seq = messageCount,
                stamp = RosUtil.GetTimeMsg(),
                frame_id = frameId
            }
        };
        List<AprilTagDetectionMsg> tagList = new List<AprilTagDetectionMsg>();
        foreach (GameObject tag in tags)
        {
            Renderer renderer = tag.GetComponent<Renderer>();
            // output only the visible renderers' name
            if (!IsVisible(renderer)) {
                continue;
            }
            float tagSize = renderer.bounds.size.x;
            LocatorTag tagInfo = tag.GetComponent<LocatorTag>();
            AprilTagDetectionMsg tagMsg = new AprilTagDetectionMsg {
                id = new int[] {tagInfo.getTagId()},
                size = new double[] {tagSize},
                pose = {
                    header = {
                        seq = messageCount,
                        stamp = RosUtil.GetTimeMsg(),
                        frame_id = frameId
                    },
                    pose = {
                        pose = tagInfo.GetTagPose(this.transform)
                    }
                }
            };
            tagList.Add(tagMsg);
        }
        messageCount++;
        if (tagTopic.Length > 0 & publishDelay > 0.0) {
            if (now - _prevPublishTime < publishDelay) {
                return;
            }
            _prevPublishTime = now;
            
            tagArrayMsg.detections = tagList.ToArray();
            _ros.Publish(tagTopic, tagArrayMsg);
        }
    }

    private bool IsVisible(Renderer renderer) 
    {
        Plane[] planes = GeometryUtility.CalculateFrustumPlanes(cameraView);

        if (GeometryUtility.TestPlanesAABB(planes, renderer.bounds)) {
            RaycastHit hit;
            Vector3 directionVector = renderer.bounds.center - cameraView.transform.position;
            var measurementStart = rayCastOffset * directionVector + transform.position;
            var measurementRay = new Ray(measurementStart, directionVector.normalized);
            if (Physics.Raycast(measurementRay, out hit)) {
                float castDistance = hit.distance + rayCastTolerance + rayCastOffset;
                bool isUnObstructed = castDistance >= directionVector.magnitude;
                if (debugRayCast) {
                    Debug.DrawRay(measurementStart, directionVector.normalized * hit.distance, isUnObstructed ? Color.green : Color.yellow);
                }
                return isUnObstructed;
            }
            return false;
        }
        else {
            return false;
        }
    }
}