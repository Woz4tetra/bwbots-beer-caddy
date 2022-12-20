using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/// <summary>
///     This script converts linear velocity and 
///     angular velocity to joint velocities for
///     differential drive robot.
/// </summary>
public class ArticulationWheelController : MonoBehaviour
{
    public ArticulationBody bodyMain;
    public ArticulationBody bodyWheelBackLeft;
    public ArticulationBody bodyWheelBackRight;
    public ArticulationBody bodyWheelFrontLeft;
    public ArticulationBody bodyWheelFrontRight;

    public ArticulationBody bodyModuleBackLeft;
    public ArticulationBody bodyModuleBackRight;
    public ArticulationBody bodyModuleFrontLeft;
    public ArticulationBody bodyModuleFrontRight;

    List<ModuleKinematics> modules;

    private const double FRONT_ANGLE = -1.2967;  // -74.293 degrees
    private const double ALCOVE_ANGLE = 0.5236;  // 30 degrees
    private const double MIN_RADIUS_OF_CURVATURE = 0.15;

    private const double WIDTH = 0.115;  // meters, chassis pivot to pivot Y dimension
    private const double LENGTH = 0.160;  // meters, chassis pivot to pivot X dimension
    private const double ARMATURE = 0.037;  // meters, pivot to wheel center dimension

    private const double MAX_WHEEL_SPEED = 1.0;  // meters/second

    private const double min_strafe_angle = -Math.PI;
    private const double max_strafe_angle = Math.PI;
    private const double reverse_min_strafe_angle = -Math.PI;
    private const double reverse_max_strafe_angle = Math.PI;

    void Start()
    {
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

        modules = new List<ModuleKinematics>();
        modules.Add(
            new ModuleKinematics(
                "back_left",
                bodyWheelBackLeft,
                bodyModuleBackLeft,
                FRONT_ANGLE,  // -75 deg
                ALCOVE_ANGLE,  // 30 deg
                MIN_RADIUS_OF_CURVATURE,
                -LENGTH / 2.0,
                -WIDTH / 2.0,
                ARMATURE,
                MAX_WHEEL_SPEED,
                true,
                true
            )
        );
        modules.Add(
            new ModuleKinematics(
                "back_right",
                bodyWheelBackRight,
                bodyModuleBackRight,
                -ALCOVE_ANGLE,  // -30 deg
                -FRONT_ANGLE,  // 75 deg
                MIN_RADIUS_OF_CURVATURE,
                -LENGTH / 2.0,
                WIDTH / 2.0,
                ARMATURE,
                MAX_WHEEL_SPEED,
                true,
                false
            )
        );
        modules.Add(
            new ModuleKinematics(
                "front_left",
                bodyWheelFrontLeft,
                bodyModuleFrontLeft,
                -ALCOVE_ANGLE,  // -30 deg
                -FRONT_ANGLE,  // 75 deg
                MIN_RADIUS_OF_CURVATURE,
                LENGTH / 2.0,
                -WIDTH / 2.0,
                ARMATURE,
                MAX_WHEEL_SPEED,
                true,
                true
            )
        );
        modules.Add(
            new ModuleKinematics(
                "front_right",
                bodyWheelFrontRight,
                bodyModuleFrontRight,
                FRONT_ANGLE,  // -75 deg
                ALCOVE_ANGLE,  // 30 deg
                MIN_RADIUS_OF_CURVATURE,
                LENGTH / 2.0,
                WIDTH / 2.0,
                ARMATURE,
                MAX_WHEEL_SPEED,
                true,
                false
            )
        );
    }

    void Update()
    {
        // double angle = Time.realtimeSinceStartupAsDouble % (2 * Math.PI);
        // Debug.Log("angle: " + angle);
        // modules[0].setModuleAzimuth(angle);
        // modules[1].setModuleAzimuth(angle);
        // modules[2].setModuleAzimuth(angle);
        // modules[3].setModuleAzimuth(angle);
        // double speed;
        // if (Time.realtimeSinceStartupAsDouble % 2.0 < 1.0) {
        //     speed = -1.0;
        // }
        // else {
        //     speed = 1.0;
        // }

        // double angle = Math.PI / 4.0;
        // // double speed = 1.0;
        // modules[0].setModuleAzimuth(-angle);
        // modules[1].setModuleAzimuth(angle);
        // modules[2].setModuleAzimuth(angle);
        // modules[3].setModuleAzimuth(-angle);
        // modules[0].setWheelVelocity(speed);
        // modules[1].setWheelVelocity(-speed);
        // modules[2].setWheelVelocity(speed);
        // modules[3].setWheelVelocity(-speed);

        // modules[0].setModuleAzimuth(0.0);
        // modules[1].setModuleAzimuth(0.0);
        // modules[2].setModuleAzimuth(0.0);
        // modules[3].setModuleAzimuth(0.0);
    }

    public void setRobotVelocity(double vx, double vy, double vt)
    {
        double v_theta = Math.Atan2(vy, vx);
        double dt = (double)Time.fixedDeltaTime;

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
        // Debug.Log(String.Format("vx: {0}, vy: {1}, vt: {2}, dt: {3}", vx, vy, vt, dt));
        foreach (ModuleKinematics module in modules) {
            module.set(vx, vy, vt, dt);
        }
    }
}
