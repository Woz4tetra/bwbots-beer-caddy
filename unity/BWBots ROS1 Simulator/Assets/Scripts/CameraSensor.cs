using System.Collections.Generic;
using RosMessageTypes.ApriltagRos;
using RosMessageTypes.Geometry;
using RosMessageTypes.Vision;
using RosMessageTypes.ZedInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

class CameraSensor : MonoBehaviour
{
    private Camera cameraView;
    private ROSConnection _ros;
    [SerializeField] private double publishDelay;
    [SerializeField] private string tagTopic;
    [SerializeField] private string detectTopic;
    [SerializeField] private string zedObjectsTopic;
    [SerializeField] private string frameId;
    [SerializeField] private float rayCastOffset = 0.01f;
    [SerializeField] private bool debugRayCast = false;
    [SerializeField] private string[] labels;
    [SerializeField] private string personLabel;
    private double _prevPublishTime;
    private uint aprilTagMessageCount;
    private uint detectionMessageCount;


    void Awake()
    {
        cameraView = GetComponent<Camera>();
        if (tagTopic.Length > 0)
        {
            _ros = ROSConnection.GetOrCreateInstance();
            _ros.RegisterPublisher<AprilTagDetectionArrayMsg>(tagTopic);
            _ros.RegisterPublisher<Detection3DArrayMsg>(detectTopic);
            _ros.RegisterPublisher<ObjectsStampedMsg>(zedObjectsTopic);
        }
    }

    void Update()
    {
        double now = Time.realtimeSinceStartup;

        GameObject[] tags = GameObject.FindGameObjectsWithTag("locator_tag");
        GameObject[] people = GameObject.FindGameObjectsWithTag("person");

        AprilTagDetectionArrayMsg tagArrayMsg = GetAprilTagArrayMsg(tags);
        Detection3DArrayMsg detectArrayMsg = GetDetectionArrayMsg(people);

        if (publishDelay > 0.0)
        {

            if (now - _prevPublishTime < publishDelay)
            {
                return;
            }
            _prevPublishTime = now;


            _ros.Publish(tagTopic, tagArrayMsg);
            _ros.Publish(detectTopic, detectArrayMsg);
            _ros.Publish(zedObjectsTopic, ConvertToZedObjects(detectArrayMsg));
        }
    }
    private Detection3DArrayMsg GetDetectionArrayMsg(GameObject[] people)
    {
        Detection3DArrayMsg detectArrayMsg = new Detection3DArrayMsg
        {
            header = {
                seq = detectionMessageCount,
                stamp = RosUtil.GetTimeMsg(),
                frame_id = frameId
            }
        };
        List<Detection3DMsg> detectList = new List<Detection3DMsg>();
        int objectCount = 0;  // A map will be required if multiple object classes are added
        int person_class_id;
        for (person_class_id = 0; person_class_id < labels.Length; person_class_id++)
        {
            if (labels[person_class_id].Equals(personLabel))
            {
                break;
            }
        }
        foreach (GameObject person in people)
        {
            if (!IsVisible(person))
            {
                continue;
            }
            Person personInfo = person.GetComponent<Person>();

            int obj_id = (objectCount << 16) | person_class_id;

            // TODO: change to bounding box that's visible
            PoseMsg pose = personInfo.GetPose(this.transform);
            ObjectHypothesisWithPoseMsg obj_hyp = new ObjectHypothesisWithPoseMsg
            {
                id = obj_id,
                score = 1.0,
                pose = new PoseWithCovarianceMsg
                {
                    pose = pose
                }
            };
            Vector3 sizeVector = person.GetComponent<Renderer>().bounds.size;
            Detection3DMsg detectMsg = new Detection3DMsg
            {
                header = detectArrayMsg.header,
                results = new ObjectHypothesisWithPoseMsg[] { obj_hyp },
                bbox = new RosMessageTypes.Vision.BoundingBox3DMsg
                {
                    center = pose,
                    size = sizeVector.To<FLU>()
                }
            };
            detectList.Add(detectMsg);
            objectCount++;
        }
        detectArrayMsg.detections = detectList.ToArray();
        detectionMessageCount++;
        return detectArrayMsg;
    }

    private ObjectsStampedMsg ConvertToZedObjects(Detection3DArrayMsg detectArrayMsg)
    {
        ObjectsStampedMsg zedObjects = new ObjectsStampedMsg
        {
            header = detectArrayMsg.header
        };
        List<ObjectMsg> objectsList = new List<ObjectMsg>();
        foreach (Detection3DMsg detectMsg in detectArrayMsg.detections)
        {
            ObjectHypothesisWithPoseMsg obj_hyp = detectMsg.results[0];
            int class_index = (int)(obj_hyp.id & 0xffff);
            if (!(0 <= class_index && class_index < labels.Length))
            {
                Debug.LogWarning($"Class index {class_index} is not in the list of labels ({labels.Length} labels)");
                continue;
            }
            string label = labels[class_index];
            ObjectMsg zedObject = new ObjectMsg
            {
                label = label,
                confidence = (float)(obj_hyp.score * 100.0),
                position = new float[] {
                    (float)obj_hyp.pose.pose.position.x,
                    (float)obj_hyp.pose.pose.position.y,
                    (float)obj_hyp.pose.pose.position.z
                },
                tracking_available = false,
                bounding_box_3d = new RosMessageTypes.ZedInterfaces.BoundingBox3DMsg
                {
                    corners = getZedKeypoints(detectMsg)
                },
                dimensions_3d = new float[] {
                    (float)detectMsg.bbox.size.x,
                    (float)detectMsg.bbox.size.y,
                    (float)detectMsg.bbox.size.z
                },
                bounding_box_2d = new BoundingBox2DiMsg
                {
                    corners = new Keypoint2DiMsg[4] {
                        new Keypoint2DiMsg {kp = new uint[2] {0, 0}},
                        new Keypoint2DiMsg {kp = new uint[2] {0, 0}},
                        new Keypoint2DiMsg {kp = new uint[2] {0, 0}},
                        new Keypoint2DiMsg {kp = new uint[2] {0, 0}}
                    }
                },
                head_bounding_box_2d = new BoundingBox2DfMsg
                {
                    corners = create_keypoint_2d_array(4)
                },
                head_bounding_box_3d = new RosMessageTypes.ZedInterfaces.BoundingBox3DMsg
                {
                    corners = create_keypoint_3d_array(8)
                },
                skeleton_2d = new Skeleton2DMsg
                {
                    keypoints = create_keypoint_2d_array(18)
                },
                skeleton_3d = new Skeleton3DMsg
                {
                    keypoints = create_keypoint_3d_array(18)
                }
            };
            objectsList.Add(zedObject);
        }
        zedObjects.objects = objectsList.ToArray();
        return zedObjects;
    }

    private Keypoint2DfMsg[] create_keypoint_2d_array(int length)
    {
        Keypoint2DfMsg[] array = new Keypoint2DfMsg[length];
        for (int index = 0; index < array.Length; index++)
        {
            array[index] = new Keypoint2DfMsg { kp = new float[2] { 0.0f, 0.0f } };
        }
        return array;
    }

    private Keypoint3DMsg[] create_keypoint_3d_array(int length)
    {
        Keypoint3DMsg[] array = new Keypoint3DMsg[length];
        for (int index = 0; index < array.Length; index++)
        {
            array[index] = new Keypoint3DMsg { kp = new float[3] { 0.0f, 0.0f, 0.0f } };
        }
        return array;
    }
    private Keypoint3DMsg[] getZedKeypoints(Detection3DMsg detectMsg)
    {
        float size_x = (float)detectMsg.bbox.size.x;
        float size_y = (float)detectMsg.bbox.size.y;
        float size_z = (float)detectMsg.bbox.size.z;
        float center_x = (float)detectMsg.bbox.center.position.x;
        float center_y = (float)detectMsg.bbox.center.position.y;
        float center_z = (float)detectMsg.bbox.center.position.z;

        float x0 = center_x - size_x / 2.0f;
        float x1 = center_x + size_x / 2.0f;
        float y0 = center_y - size_y / 2.0f;
        float y1 = center_y + size_y / 2.0f;
        float z0 = center_z - size_z / 2.0f;
        float z1 = center_z + size_z / 2.0f;

        return new Keypoint3DMsg[8] {
            new Keypoint3DMsg { kp = new float[3] {x0, y0, z0} },
            new Keypoint3DMsg { kp = new float[3] {x1, y0, z0} },
            new Keypoint3DMsg { kp = new float[3] {x1, y1, z0} },
            new Keypoint3DMsg { kp = new float[3] {x0, y1, z0} },
            new Keypoint3DMsg { kp = new float[3] {x0, y0, z1} },
            new Keypoint3DMsg { kp = new float[3] {x1, y0, z1} },
            new Keypoint3DMsg { kp = new float[3] {x1, y1, z1} },
            new Keypoint3DMsg { kp = new float[3] {x0, y1, z1} }
        };
    }

    private AprilTagDetectionArrayMsg GetAprilTagArrayMsg(GameObject[] tags)
    {
        AprilTagDetectionArrayMsg tagArrayMsg = new AprilTagDetectionArrayMsg
        {
            header = {
                seq = aprilTagMessageCount,
                stamp = RosUtil.GetTimeMsg(),
                frame_id = frameId
            }
        };
        List<AprilTagDetectionMsg> tagList = new List<AprilTagDetectionMsg>();
        foreach (GameObject tag in tags)
        {
            if (!IsVisible(tag))
            {
                continue;
            }
            float tagSize = tag.GetComponent<Renderer>().bounds.size.x;
            LocatorTag tagInfo = tag.GetComponent<LocatorTag>();
            AprilTagDetectionMsg tagMsg = new AprilTagDetectionMsg
            {
                id = new int[] { tagInfo.getTagId() },
                size = new double[] { tagSize },
                pose = {
                    header = {
                        seq = aprilTagMessageCount,
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
        tagArrayMsg.detections = tagList.ToArray();
        aprilTagMessageCount++;
        return tagArrayMsg;
    }

    private bool IsVisible(GameObject gameObj)
    {
        Renderer renderer = gameObj.GetComponent<Renderer>();
        Plane[] planes = GeometryUtility.CalculateFrustumPlanes(cameraView);

        if (GeometryUtility.TestPlanesAABB(planes, renderer.bounds))
        {
            RaycastHit hit;
            Vector3 directionVector = renderer.bounds.center - cameraView.transform.position;
            var measurementStart = rayCastOffset * directionVector + transform.position;
            var measurementRay = new Ray(measurementStart, directionVector.normalized);
            if (Physics.Raycast(measurementRay, out hit))
            {
                bool isUnObstructed = IsChild(GetTopLevelObject(hit.transform.gameObject), gameObj);
                if (debugRayCast)
                {
                    Debug.DrawRay(measurementStart, directionVector.normalized * hit.distance, isUnObstructed ? Color.green : Color.yellow);
                }
                return isUnObstructed;
            }
            return false;
        }
        else
        {
            return false;
        }
    }

    private GameObject GetTopLevelObject(GameObject obj)
    {
        Transform tf = obj.transform;
        while (true)
        {
            if (tf.parent == null)
            {
                break;
            }
            tf = tf.parent;
        }
        return tf.gameObject;
    }

    private bool IsChild(GameObject parent, GameObject check)
    {
        if (parent == check)
        {
            return true;
        }
        Transform child = null;
        for (int i = 0; i < parent.transform.childCount; i++)
        {
            child = parent.transform.GetChild(i);
            if (child.gameObject == check)
            {
                return true;
            }
            else
            {
                bool found = IsChild(child.gameObject, check);
                if (found)
                {
                    return true;
                }
            }
        }

        return false;
    }
}