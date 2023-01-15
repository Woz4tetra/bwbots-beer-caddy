using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;


public class Person : MonoBehaviour
{
    [SerializeField] private GameObject robot;

    private Collider pickup_collision;
    private Collider robot_collision;
    private BwbotsSimulatedChassis robot_chassis;

    private const double min_linear_vel = 0.1;
    private const double min_angular_vel = 0.2;
    private const float settle_time_threshold = 1.0f;

    private float prev_settle_time = 0.0f;

    void Start()
    {
        pickup_collision = FindComponentInChildWithTag<Collider>("pickup");
        robot_collision = robot.GetComponent<Collider>();
        robot_chassis = robot.GetComponent<BwbotsSimulatedChassis>();
    }

    private T FindComponentInChildWithTag<T>(string tag) where T : Component
    {
        Transform t = this.transform;
        foreach (Transform tr in t)
        {
            if (tr.tag == tag)
            {
                return tr.GetComponent<T>();
            }
        }
        return null;
    }

    private void deleteBeerCans()
    {
        foreach (GameObject can in GameObject.FindGameObjectsWithTag("beer_can"))
        {
            Collider can_collision = can.GetComponent<Collider>();
            if (pickup_collision.bounds.Intersects(can_collision.bounds))
            {
                Destroy(can);
            }
        }
    }

    void FixedUpdate()
    {
        if (pickup_collision.bounds.Intersects(robot_collision.bounds))
        {
            float current_time = Time.realtimeSinceStartup;
            OdometryMsg msg = robot_chassis.GetOdometryMessage();
            double vx = msg.twist.twist.linear.x;
            double vy = msg.twist.twist.linear.y;
            double vt = msg.twist.twist.angular.z;
            if (Math.Abs(vx) < min_linear_vel &&
                Math.Abs(vy) < min_linear_vel &&
                Math.Abs(vt) < min_angular_vel)
            {
                if (prev_settle_time == 0.0f)
                {
                    prev_settle_time = current_time;
                }
            }
            else
            {
                prev_settle_time = 0.0f;
            }

            if (prev_settle_time != 0.0f && current_time - prev_settle_time > settle_time_threshold)
            {
                deleteBeerCans();
            }
        }
        else
        {
            prev_settle_time = 0.0f;
        }
    }

    public PoseMsg GetPose(Transform cameraTransform)
    {
        Vector3 relativePoint = cameraTransform.InverseTransformPoint(GetComponent<Renderer>().bounds.center);
        Quaternion LocalRotation = Quaternion.Inverse(cameraTransform.rotation) * transform.rotation;
        LocalRotation = LocalRotation * Quaternion.AngleAxis(-90.0f, Vector3.up);
        PoseMsg pose = new PoseMsg();
        pose.position = relativePoint.To<FLU>();
        pose.orientation = LocalRotation.To<FLU>();
        return pose;
    }
}
