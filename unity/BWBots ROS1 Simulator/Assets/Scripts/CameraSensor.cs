using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Std;
using RosMessageTypes.ApriltagRos;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
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
    [SerializeField] private string zedCloudTopic;
    [SerializeField] private string tagRequestTopic;
    [SerializeField] private string tagResponseTopic;
    [SerializeField] private string frameId;
    [SerializeField] private float rayCastOffset = 0.01f;
    [SerializeField] private bool debugRayCast = false;
    [SerializeField] private string[] labels;
    [SerializeField] private string personLabel;
    [SerializeField] private bool enablePointCloud = false;
    [SerializeField] private uint pointCloudNumVerticalRays = 10;
    [SerializeField] private uint pointCloudNumHorizontalRays = 10;
    [SerializeField] private float[] pointCloudWindow;
    [SerializeField] private float noiseMagnitude = 0.05f;
    [SerializeField] private float RangeMetersMax = 1000;
    [SerializeField] private string locatorTag = "locator_tag";
    [SerializeField] private string personTag = "person";
    private double _prevPublishTime;
    private uint aprilTagMessageCount;
    private uint detectionMessageCount;
    private PointCloud2Msg cloudMsg;
    private float[] pointCloudXIterator;
    private float[] pointCloudYIterator;
    private uint prevScreenSize = 0;


    void Start()
    {
        cameraView = GetComponent<Camera>();
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<AprilTagDetectionArrayMsg>(tagTopic);
        _ros.RegisterPublisher<Detection3DArrayMsg>(detectTopic);
        _ros.RegisterPublisher<ObjectsStampedMsg>(zedObjectsTopic);
        if (enablePointCloud)
        {
            _ros.RegisterPublisher<PointCloud2Msg>(zedCloudTopic);
            cloudMsg = makeEmptyCloudMsg(pointCloudNumHorizontalRays, pointCloudNumVerticalRays);
        }
        _ros.Subscribe<StringMsg>(tagRequestTopic, tagRequestCallback);
        _ros.RegisterPublisher<AprilTagDetectionArrayMsg>(tagResponseTopic);
    }

    void Update()
    {
        double now = Time.realtimeSinceStartup;

        GameObject[] tags = GameObject.FindGameObjectsWithTag(locatorTag);
        GameObject[] people = GameObject.FindGameObjectsWithTag(personTag);

        AprilTagDetectionArrayMsg tagArrayMsg = GetAprilTagArrayMsg(tags);
        Detection3DArrayMsg detectArrayMsg = GetDetectionArrayMsg(people);

        PointCloud2Msg cloud = GeneratePointCloud();

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
            if (enablePointCloud)
            {
                _ros.Publish(zedCloudTopic, cloud);
            }
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
            Vector3Msg sizeMsg = sizeVector.To<FLU>();
            Detection3DMsg detectMsg = new Detection3DMsg
            {
                header = detectArrayMsg.header,
                results = new ObjectHypothesisWithPoseMsg[] { obj_hyp },
                bbox = new RosMessageTypes.Vision.BoundingBox3DMsg
                {
                    center = pose,
                    size = new Vector3Msg
                    {
                        x = Math.Abs(sizeMsg.x),
                        y = Math.Abs(sizeMsg.y),
                        z = Math.Abs(sizeMsg.z),
                    }
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

    void OnDrawGizmos()
    {
        // Gizmo Frustum
        if (cameraView && debugRayCast)
        {
            Gizmos.matrix = transform.localToWorldMatrix;           // For the rotation bug
            Gizmos.DrawFrustum(transform.position, cameraView.fieldOfView, cameraView.nearClipPlane, cameraView.farClipPlane, cameraView.aspect);
        }
    }

    private PointCloud2Msg makeEmptyCloudMsg(uint numHorizontalRays, uint numVerticalRays)
    {
        PointCloud2Msg cloud = new PointCloud2Msg();
        cloud.width = numHorizontalRays;
        cloud.height = numVerticalRays;
        cloud.header.frame_id = frameId;
        cloud.is_bigendian = false;
        cloud.is_dense = false;

        uint data_size_bytes = 4;
        uint num_fields = 3;
        cloud.point_step = data_size_bytes * num_fields;
        cloud.row_step = numHorizontalRays * cloud.point_step;

        cloud.fields = new PointFieldMsg[num_fields];
        uint field_index = 0;
        PointFieldMsg xfield = new PointFieldMsg();
        xfield.datatype = PointFieldMsg.FLOAT32;
        xfield.name = "x";
        xfield.offset = 0;
        xfield.count = 1;

        PointFieldMsg yfield = new PointFieldMsg();
        yfield.datatype = PointFieldMsg.FLOAT32;
        yfield.name = "y";
        yfield.offset = 4;
        yfield.count = 1;

        PointFieldMsg zfield = new PointFieldMsg();
        zfield.datatype = PointFieldMsg.FLOAT32;
        zfield.name = "z";
        zfield.offset = 8;
        zfield.count = 1;

        cloud.fields[field_index++] = xfield;
        cloud.fields[field_index++] = yfield;
        cloud.fields[field_index++] = zfield;

        uint data_length = numHorizontalRays * numVerticalRays * cloud.point_step;
        cloud.data = new byte[data_length];

        return cloud;
    }

    private PointCloud2Msg GeneratePointCloud()
    {
        if (!enablePointCloud)
        {
            return cloudMsg;
        }
        uint screenSize = (uint)(Screen.width * Screen.height);
        if (screenSize != prevScreenSize)
        {
            prevScreenSize = screenSize;
            float x0, y0, x1, y1;
            x0 = 0.0f;
            y0 = Screen.width;
            x1 = 0.0f;
            y1 = Screen.height;
            if (pointCloudWindow.Length == 4)
            {
                x0 = Screen.width * pointCloudWindow[0];
                x1 = Screen.width * pointCloudWindow[1];
                y0 = Screen.height * pointCloudWindow[2];
                y1 = Screen.height * pointCloudWindow[3];
            }
            pointCloudXIterator = LinearSpacedIndices(x0, x1, (int)pointCloudNumHorizontalRays);
            pointCloudYIterator = LinearSpacedIndices(y0, y1, (int)pointCloudNumVerticalRays);
        }

        cloudMsg.header.stamp = RosUtil.GetTimeMsg();
        uint data_index = 0;
        foreach (float xindex in pointCloudXIterator)
        {
            foreach (float yindex in pointCloudYIterator)
            {
                Ray ray = cameraView.ScreenPointToRay(new Vector3(xindex, yindex, 0.0f));
                float x, y, z;
                RaycastHit hit;
                if (Physics.Raycast(ray, out hit) && hit.distance <= RangeMetersMax)
                {

                    if (debugRayCast)
                    {
                        Debug.DrawRay(ray.origin, ray.direction * hit.distance, Color.blue);
                    }

                    Vector3 relativePoint = cameraView.transform.InverseTransformPoint(hit.point);

                    Vector3Msg coord = relativePoint.To<FLU>();
                    if (noiseMagnitude == 0.0f)
                    {
                        x = (float)coord.x;
                        y = (float)coord.y;
                        z = (float)coord.z;
                    }
                    else
                    {
                        float magnitude = noiseMagnitude * hit.distance / RangeMetersMax;
                        x = (float)coord.x + UnityEngine.Random.Range(-magnitude, magnitude);
                        y = (float)coord.y + UnityEngine.Random.Range(-magnitude, magnitude);
                        z = (float)coord.z + UnityEngine.Random.Range(-magnitude, magnitude);
                    }
                }
                else
                {
                    x = float.PositiveInfinity;
                    y = float.PositiveInfinity;
                    z = float.PositiveInfinity;
                }

                byte[] xbytes = floatToBytes(x);
                byte[] ybytes = floatToBytes(y);
                byte[] zbytes = floatToBytes(z);
                for (uint byte_index = 0; byte_index < xbytes.Length; byte_index++)
                {
                    cloudMsg.data[data_index++] = xbytes[byte_index];
                }
                for (uint byte_index = 0; byte_index < ybytes.Length; byte_index++)
                {
                    cloudMsg.data[data_index++] = ybytes[byte_index];
                }
                for (uint byte_index = 0; byte_index < zbytes.Length; byte_index++)
                {
                    cloudMsg.data[data_index++] = zbytes[byte_index];
                }
            }
        }
        if (data_index != cloudMsg.data.Length)
        {
            Debug.LogWarning($"Expected data length doesn't match calculated data length! {cloudMsg.data.Length} != {data_index}");
        }
        return cloudMsg;
    }

    private static byte[] floatToBytes(float value)
    {
        var byteArray = new byte[4];
        var floatArray = new float[1];
        floatArray[0] = value;
        Buffer.BlockCopy(floatArray, 0, byteArray, 0, byteArray.Length);
        return byteArray;

    }

    public static float[] LinearSpacedIndices(float startval, float endval, int steps)
    {
        float interval = (endval / Mathf.Abs(endval)) * Mathf.Abs(endval - startval) / (steps - 1);
        return (from val in Enumerable.Range(0, steps)
                select startval + (val * interval)).ToArray();
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

    private void tagRequestCallback(StringMsg msg)
    {
        GameObject[] tags = GameObject.FindGameObjectsWithTag(locatorTag);
        AprilTagDetectionArrayMsg tagArrayMsg = GetAprilTagArrayMsg(tags);
        _ros.Publish(tagTopic, tagArrayMsg);
        _ros.Publish(tagResponseTopic, tagArrayMsg);
    }
}
