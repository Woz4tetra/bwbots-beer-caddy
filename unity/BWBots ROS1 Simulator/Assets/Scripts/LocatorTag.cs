using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

class LocatorTag : MonoBehaviour {
    [SerializeField] private string tagName;
    [SerializeField] private int tagId;
    public PoseMsg GetTagPose(Transform cameraTransform) {
        Vector3 relativePoint = cameraTransform.InverseTransformPoint(transform.position);
        Quaternion LocalRotation = Quaternion.Inverse(cameraTransform.rotation) * transform.rotation;
        LocalRotation = LocalRotation * Quaternion.AngleAxis(-90.0f, Vector3.up);
        PoseMsg pose = new PoseMsg();
        pose.position = relativePoint.To<FLU>();
        pose.orientation = LocalRotation.To<FLU>();
        return pose;
    }
    public string getTagName() {
        return tagName;
    }
    public int getTagId() {
        return tagId;
    }
}
