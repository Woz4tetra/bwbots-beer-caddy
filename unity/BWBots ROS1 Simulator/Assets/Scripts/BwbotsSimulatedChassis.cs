using System;
using UnityEngine;
using RosMessageTypes.Nav;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.BwInterfaces;

/// <summary>
///     This script converts linear velocity and 
///     angular velocity to joint velocities for
///     differential drive robot.
/// </summary>
public class BwbotsSimulatedChassis : MonoBehaviour
{
    [SerializeField] private GameObject robot;
    [SerializeField] private GameObject wheelBackLeft;
    [SerializeField] private GameObject wheelBackRight;
    [SerializeField] private GameObject wheelFrontLeft;
    [SerializeField] private GameObject wheelFrontRight;

    [SerializeField] private GameObject moduleBackLeft;
    [SerializeField] private GameObject moduleBackRight;
    [SerializeField] private GameObject moduleFrontLeft;
    [SerializeField] private GameObject moduleFrontRight;
    [SerializeField] private bool useGroundTruth;

    private ArticulationBody bodyMain;
    private Transform initialTransform;

    private List<ModuleKinematics> modules;

    private const double FRONT_ANGLE = -1.2967;  // -74.293 degrees
    private const double ALCOVE_ANGLE = 0.5236;  // 30 degrees
    private const double MIN_RADIUS_OF_CURVATURE = 0.15;
    private const double WHEEL_RADIUS = 0.115 / 2.0;  // meters

    private const double WIDTH = 0.115;  // meters, chassis pivot to pivot Y dimension
    private const double LENGTH = 0.160;  // meters, chassis pivot to pivot X dimension
    private const double ARMATURE = 0.037;  // meters, pivot to wheel center dimension

    private const double MAX_WHEEL_SPEED = 1.0;  // meters/second

    private double min_strafe_angle = -Math.PI;
    private double max_strafe_angle = Math.PI;
    private double reverse_min_strafe_angle = -Math.PI;
    private double reverse_max_strafe_angle = Math.PI;

    private double[] odom_covariance;
    private double[] twist_covariance;
    private OdometryMsg last_odom_msg;
    private double prevOdomX = 0.0, prevOdomY = 0.0, prevOdomTheta = 0.0;
    private uint odomMessageCount = 0;

    private const int CHASSIS_STATE_LEN = 3;
    private int kinematics_channels_len;
    private double[] inverse_kinematics;
    private double[] forward_kinematics;
    private double[] ik_transpose;
    private double[] kinematics_temp;
    private double[] chassis_vector;
    private double[] module_vector;

    private double[] prev_position;
    private TwistMsg twistCommand = new TwistMsg();
    private int commandPriority = -1;
    private float commandTimeoutStamp = 0.0f;
    private double velocityLimit = 1000.0;
    private double angularVelocityLimit = 1000.0;
    private double positionLimit = 10000.0;
    private bool motorsEnabled = false;


    public BwbotsSimulatedChassis() {
        modules = new List<ModuleKinematics>();
    }

    void Start()
    {
        bodyMain = robot.GetComponent<ArticulationBody>();
        ArticulationDrive bodyDrive = bodyMain.xDrive;
        bodyDrive.stiffness = 200.0f;
        bodyDrive.damping = 25.0f;
        bodyDrive.forceLimit = 3.402823e+38f;
        bodyMain.xDrive = bodyDrive;

        bodyMain.linearDamping = 1.0f;
        bodyMain.angularDamping = 1.0f;
        bodyMain.jointFriction = 0.1f;
        bodyMain.inertiaTensor = new Vector3(2e-07f, 2e-07f, 2e-07f);
        bodyMain.mass = 0.005f;

        max_strafe_angle = ALCOVE_ANGLE;
        min_strafe_angle = -ALCOVE_ANGLE;
        reverse_max_strafe_angle = Math.PI - max_strafe_angle;
        reverse_min_strafe_angle = -Math.PI - min_strafe_angle;

        modules.Add(
            new ModuleKinematics(
                "back_left",
                0,
                wheelBackLeft,
                moduleBackLeft,
                -ALCOVE_ANGLE,  // -30 deg
                -FRONT_ANGLE,  // 75 deg
                WHEEL_RADIUS,
                MIN_RADIUS_OF_CURVATURE,
                -LENGTH / 2.0,
                WIDTH / 2.0,
                ARMATURE,
                MAX_WHEEL_SPEED
            )
        );
        modules.Add(
            new ModuleKinematics(
                "back_right",
                1,
                wheelBackRight,
                moduleBackRight,
                FRONT_ANGLE,  // -75 deg
                ALCOVE_ANGLE,  // 30 deg
                WHEEL_RADIUS,
                MIN_RADIUS_OF_CURVATURE,
                -LENGTH / 2.0,
                -WIDTH / 2.0,
                ARMATURE,
                MAX_WHEEL_SPEED
            )
        );
        modules.Add(
            new ModuleKinematics(
                "front_left",
                2,
                wheelFrontLeft,
                moduleFrontLeft,
                FRONT_ANGLE,  // -75 deg
                ALCOVE_ANGLE,  // 30 deg
                WHEEL_RADIUS,
                MIN_RADIUS_OF_CURVATURE,
                LENGTH / 2.0,
                WIDTH / 2.0,
                ARMATURE,
                MAX_WHEEL_SPEED
            )
        );
        modules.Add(
            new ModuleKinematics(
                "front_right",
                3,
                wheelFrontRight,
                moduleFrontRight,
                -ALCOVE_ANGLE,  // -30 deg
                -FRONT_ANGLE,  // 75 deg
                WHEEL_RADIUS,
                MIN_RADIUS_OF_CURVATURE,
                LENGTH / 2.0,
                -WIDTH / 2.0,
                ARMATURE,
                MAX_WHEEL_SPEED
            )
        );

        kinematics_channels_len = 2 * modules.Count;
        inverse_kinematics = SimpleMatrix.MakeEmptyMatrix(kinematics_channels_len, CHASSIS_STATE_LEN);
        forward_kinematics = SimpleMatrix.MakeEmptyMatrix(CHASSIS_STATE_LEN, kinematics_channels_len);
        ik_transpose = SimpleMatrix.MakeEmptyMatrix(CHASSIS_STATE_LEN, kinematics_channels_len);
        kinematics_temp = SimpleMatrix.MakeEmptyMatrix(CHASSIS_STATE_LEN, CHASSIS_STATE_LEN);
        chassis_vector = SimpleMatrix.MakeEmptyMatrix(CHASSIS_STATE_LEN, 1);
        module_vector = SimpleMatrix.MakeEmptyMatrix(kinematics_channels_len, 1);

        for (int index = 0; index < kinematics_channels_len * CHASSIS_STATE_LEN; index++) {
            inverse_kinematics[index] = 0.0;
            forward_kinematics[index] = 0.0;
            ik_transpose[index] = 0.0;
        }
        for (int index = 0; index < CHASSIS_STATE_LEN * CHASSIS_STATE_LEN; index++) {
            kinematics_temp[index] = 0.0;
        }
        for (int index = 0; index < CHASSIS_STATE_LEN; index++) {
            chassis_vector[index] = 0.0;
        }
        for (int index = 0; index < kinematics_channels_len; index++) {
            module_vector[index] = 0.0;
        }

        for (int channel = 0; channel < modules.Count; channel++) {
            int y_row = CHASSIS_STATE_LEN * (2 * channel);
            int x_row = CHASSIS_STATE_LEN * (2 * channel + 1);
            //  0  1  2 -- channel 0, -y0
            //  3  4  5 -- channel 0, x0
            //  6  7  8 -- channel 1, -y1
            //  9 10 11 -- channel 1, x1
            // 12 13 14 -- channel 2, -y2
            // 15 16 17 -- channel 2, x2
            // 18 19 20 -- channel 3, -y3
            // 21 22 23 -- channel 3, x3

            inverse_kinematics[y_row + 0] = 1.0;
            inverse_kinematics[y_row + 1] = 0.0;
            inverse_kinematics[y_row + 2] = 0.0;  // filled in later
            inverse_kinematics[x_row + 0] = 0.0;
            inverse_kinematics[x_row + 1] = 1.0;
            inverse_kinematics[x_row + 2] = 0.0;  // filled in later
        }

        prev_position = new double[3];
        for (int index = 0; index < prev_position.Length; index++) {
            prev_position[index] = 0.0;
        }

        odom_covariance = new double[] {
            1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
        };
        twist_covariance = new double[] {
            1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
        };

        last_odom_msg = MakeOdometryMessage(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        setUseGroundTruth(useGroundTruth);
    }

    void FixedUpdate()
    {
        double vx, vy, vt;
        double x, y, theta;
        if (useGroundTruth) {
            x = bodyMain.transform.position.z;
            y = -bodyMain.transform.position.x;
            theta = ModuleKinematics.WrapAngle(-bodyMain.transform.rotation.eulerAngles.y * Mathf.Deg2Rad);

            double dt = Time.fixedDeltaTime;

            double dx = x - prevOdomX;
            double dy = y - prevOdomY;
            double dtheta = theta - prevOdomTheta;
            
            double rotatedDx = dx * Math.Cos(-theta) - dy * Math.Sin(-theta);
            double rotatedDy = dx * Math.Sin(-theta) + dy * Math.Cos(-theta);

            vx = rotatedDx / dt;
            vy = rotatedDy / dt;
            vt = dtheta / dt;

            prevOdomX = x;
            prevOdomY = y;
            prevOdomTheta = theta;
        }
        else {
            double dt = (double)Time.fixedDeltaTime;
            if (motorsEnabled) {
                (vx, vy, vt) = computeVelocity();
            }
            else {
                vx = 0.0;
                vy = 0.0;
                vt = 0.0;
            }
            (x, y, theta) = computePosition(vx, vy, vt, dt);
        }
        last_odom_msg = MakeOdometryMessage(x, y, theta, vx, vy, vt);

        setRobotVelocity(twistCommand.linear.x, twistCommand.linear.y, twistCommand.angular.z);

        if (Math.Abs(vx) > velocityLimit || Math.Abs(vy) > velocityLimit || Math.Abs(vt) > angularVelocityLimit
                || Math.Abs(x) > positionLimit || Math.Abs(y) > positionLimit) {
            bodyMain.TeleportRoot(Vector3.zero, Quaternion.identity);
            Debug.LogWarning($"Robot exceeds spatial constaints {last_odom_msg}! Teleporting to origin.");
        }
    }

    public bool getUseGroundTruth() {
        return this.useGroundTruth;
    }
    public void setUseGroundTruth(bool useGroundTruth) {
        this.useGroundTruth = useGroundTruth;
        foreach (ModuleKinematics module in modules) {
            module.setUseGroundTruth(useGroundTruth);
        }
        if (this.useGroundTruth) {
            initialTransform = bodyMain.transform;
        }
    }

    public void setTwistCommand(TwistMsg twist, int priority, float priorityTimeout) {
        float now = Time.realtimeSinceStartup;
        if (commandPriority == -1 || priority <= commandPriority || now > commandTimeoutStamp) {
            twistCommand = twist;
            commandPriority = priority;
            commandTimeoutStamp = now + priorityTimeout;
        }
    }

    public void setMotorEnable(bool enabled) {
        motorsEnabled = enabled;
    }

    public bool getMotorEnable() {
        return motorsEnabled;
    }

    private void setRobotVelocity(double vx, double vy, double vt)
    {
        if (!motorsEnabled) {
            vx = 0.0;
            vy = 0.0;
            vt = 0.0;
        }

        
        double v_theta = Math.Atan2(vy, vx);
        double dt = (double)Time.unscaledDeltaTime;

        if ((v_theta > max_strafe_angle && v_theta < reverse_max_strafe_angle) || 
                (v_theta < min_strafe_angle && v_theta > reverse_min_strafe_angle)) {
            double v_mag = Math.Sqrt(vx * vx + vy * vy);
            if (0.0 <= v_theta && v_theta < Math.PI / 2.0) {
                v_theta = max_strafe_angle;
            }
            else if (Math.PI / 2.0 <= v_theta && v_theta <= Math.PI) {
                v_theta = reverse_max_strafe_angle;
            }
            else if (-Math.PI / 2.0 <= v_theta && v_theta < 0.0) {
                v_theta = min_strafe_angle;
            }
            else if (-Math.PI <= v_theta && v_theta < -Math.PI / 2.0) {
                v_theta = reverse_min_strafe_angle;
            }
            
            vx = v_mag * Math.Cos(v_theta);
            vy = v_mag * Math.Sin(v_theta);
        }

        if (useGroundTruth) {
            // Debug.Log($"vx: {vx}, vy: {vy}, vt: {vt}");
            float unityDz = (float)vx * Time.fixedDeltaTime;
            float unityDx = -(float)vy * Time.fixedDeltaTime;
            float unityDtheta = -(float)(vt * Mathf.Rad2Deg) * Time.fixedDeltaTime;
            Vector3 deltaVector = new Vector3(unityDx, 0.0f, unityDz);
            Vector3 rotatedVector = bodyMain.transform.position + bodyMain.transform.rotation * deltaVector;
            rotatedVector.y = initialTransform.position.y;
            Quaternion newRotation = Quaternion.Euler(0.0f, bodyMain.transform.rotation.eulerAngles.y + unityDtheta, 0.0f);
            bodyMain.TeleportRoot(
                rotatedVector,
                newRotation
            );
        }
        else {
            foreach (ModuleKinematics module in modules) {
                module.set(vx, vy, vt, dt);
            }
        }
    }

    public OdometryMsg GetOdometryMessage(){
        return last_odom_msg;
    }

    private OdometryMsg MakeOdometryMessage(double x, double y, double theta, double vx, double vy, double vt) {
        OdometryMsg msg = new OdometryMsg();
        msg.header.frame_id = "odom";
        msg.header.stamp = RosUtil.GetTimeMsg();
        msg.header.seq = odomMessageCount;
        msg.child_frame_id = "base_link";
        Quaternion quat = Quaternion.Euler(0.0f, (float)(-theta * Mathf.Rad2Deg), 0.0f);
        PoseMsg pose = new PoseMsg(
            new PointMsg(x, y, 0.0),
            quat.To<FLU>()
        );
        TwistMsg twist = new TwistMsg(
            new Vector3Msg(vx, vy, 0.0),
            new Vector3Msg(0.0, 0.0, vt)
        );

        msg.pose = new PoseWithCovarianceMsg(pose, odom_covariance);
        msg.twist = new TwistWithCovarianceMsg(twist, twist_covariance);
        odomMessageCount++;
        return msg;
    }

    private (double, double, double) computeVelocity() {
        int channel = 0;
        foreach (ModuleKinematics module in modules) {
            double azimuth = module.getAzimuth();
            double geometric_azimuth = 0.0;
            if (module.getYLocation() < 0.0) {
                geometric_azimuth = azimuth * Math.PI;
            }
            else {
                geometric_azimuth = azimuth;
            }
            double x = module.getXLocation() + 2 * ARMATURE * Math.Sin(geometric_azimuth);
            double y = module.getYLocation() + 2 * ARMATURE * Math.Cos(geometric_azimuth);

            //  0  1  2 -- channel 0, -y0
            //  3  4  5 -- channel 0, x0
            //  6  7  8 -- channel 1, -y1
            //  9 10 11 -- channel 1, x1
            // 12 13 14 -- channel 2, -y2
            // 15 16 17 -- channel 2, x2
            // 18 19 20 -- channel 3, -y3
            // 21 22 23 -- channel 3, x3
            int y_index = CHASSIS_STATE_LEN * (2 * channel) + 2;
            int x_index = CHASSIS_STATE_LEN * (2 * channel + 1) + 2;
            inverse_kinematics[y_index] = -y;
            inverse_kinematics[x_index] = x;
            
            double wheel_velocity = module.getWheelVelocity();
            double module_vx = wheel_velocity * Math.Cos(azimuth);
            double module_vy = wheel_velocity * Math.Sin(azimuth);
            int even_index = 2 * channel;
            int odd_index = 2 * channel + 1;
            module_vector[even_index] = module_vx;
            module_vector[odd_index] = module_vy;

            channel++;
        }
        // SimpleMatrix.Print(inverse_kinematics, kinematics_channels_len, CHASSIS_STATE_LEN, "M");

        // ik_transpose = M'
        SimpleMatrix.Transpose(
            inverse_kinematics,  // source
            kinematics_channels_len,  // rows
            CHASSIS_STATE_LEN,  // columns
            ik_transpose  // destination
        );
        // SimpleMatrix.Print(ik_transpose, CHASSIS_STATE_LEN, kinematics_channels_len, "M.T");

        // kinematics_temp = M' * M
        SimpleMatrix.Multiply(
            ik_transpose,
            inverse_kinematics,
            CHASSIS_STATE_LEN,  // rows of ik_transpose
            kinematics_channels_len,  // columns of ik_transpose / rows of inverse_kinematics
            CHASSIS_STATE_LEN,  // columns of inverse_kinematics
            kinematics_temp  // destination
        );
        // SimpleMatrix.Print(kinematics_temp, CHASSIS_STATE_LEN, CHASSIS_STATE_LEN, "M' * M");

        // kinematics_temp = (M' * M)^-1
        if (!SimpleMatrix.Invert(kinematics_temp, CHASSIS_STATE_LEN)) {
            Debug.LogWarning("Failed to invert matrix for velocity calculation!");
            return (Double.NaN, Double.NaN, Double.NaN);
        }

        // forward_kinematics = (M' * M)^-1 * M' = pinv(M)
        SimpleMatrix.Multiply(
            kinematics_temp,
            ik_transpose,
            CHASSIS_STATE_LEN,  // rows of kinematics_temp
            CHASSIS_STATE_LEN,  // columns of ik_transpose / rows of kinematics_temp
            kinematics_channels_len,  // columns of ik_transpose
            forward_kinematics  // destination
        );
        // SimpleMatrix.Print(kinematics_temp, CHASSIS_STATE_LEN, CHASSIS_STATE_LEN, "(M' * M)^-1");

        // chassis_vector = pinv(M) * module_vector
        SimpleMatrix.Multiply(
            forward_kinematics,
            module_vector,
            CHASSIS_STATE_LEN,  // rows of forward_kinematics
            kinematics_channels_len,  // columns of forward_kinematics / rows of module_vector
            1,  // columns of module_vector
            chassis_vector  // destination
        );
        // SimpleMatrix.Print(forward_kinematics, CHASSIS_STATE_LEN, kinematics_channels_len, "(M' * M)^-1 * M'");

        // SimpleMatrix.Print(module_vector, kinematics_channels_len, 1, "mv");
        // SimpleMatrix.Print(chassis_vector, CHASSIS_STATE_LEN, 1, "pinv(M) * mv");

        double vx = chassis_vector[0];
        double vy = chassis_vector[1];
        double vt = chassis_vector[2];

        // Debug.Log($"vx: {vx}, vy: {vy}, vt: {vt}");

        return (vx, vy, vt);
    }

    private (double, double, double) computePosition(double vx, double vy, double vt, double dt) {
        double dx = vx * dt;
        double dy = vy * dt;
        double dtheta = vt * dt;
        double sin_theta = Math.Sin(dtheta);
        double cos_theta = Math.Cos(dtheta);

        double s, c;
        if (Math.Abs(dtheta) < 1E-9) {  // if angle is too small, use taylor series linear approximation
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        }
        else {
            s = sin_theta / dtheta;
            c = (1.0 - cos_theta) / dtheta;
        }

        double tx = dx * s - dy * c;
        double ty = dx * c + dy * s;

        double theta = prev_position[2];
        double rotated_tx = tx * Math.Cos(theta) - ty * Math.Sin(theta);
        double rotated_ty = tx * Math.Sin(theta) + ty * Math.Cos(theta);

        prev_position[0] += rotated_tx;
        prev_position[1] += rotated_ty;
        prev_position[2] += dtheta;

        return (prev_position[0], prev_position[1], prev_position[2]);
    }

    public int getNumModules() {
        return 4;
    }

    public BwDriveModuleMsg getModuleMessage(int index) {
        return this.modules[index].getMessage();
    }
}
