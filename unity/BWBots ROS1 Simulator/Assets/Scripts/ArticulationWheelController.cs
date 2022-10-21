using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
///     This script converts linear velocity and 
///     angular velocity to joint velocities for
///     differential drive robot.
/// </summary>
public class ArticulationWheelController : MonoBehaviour
{
    public ArticulationBody wheel1;
    public ArticulationBody wheel2;
    public ArticulationBody wheel3;
    public ArticulationBody wheel4;

    private float vel1;
    private float vel2;
    private float vel3;
    private float vel4;

    void Start()
    {
    }

    void Update()
    {
    }

    public void setRobotVelocity(float targetLinearSpeed, float targetLateralSpeed, float targetAngularSpeed)
    {
        setWheelVelocity(wheel1, targetLinearSpeed);
        setWheelVelocity(wheel2, -targetLinearSpeed);
        setWheelVelocity(wheel3, targetLinearSpeed);
        setWheelVelocity(wheel4, -targetLinearSpeed);
    }

    private void setWheelVelocity(ArticulationBody wheel, float jointVelocity)
    {
        ArticulationDrive drive = wheel.xDrive;
        drive.target = drive.target + jointVelocity * Time.fixedDeltaTime;
        wheel.xDrive = drive;
    }

    private void stopWheel(ArticulationBody wheel)
    {
        // Set desired angle as current angle to stop the wheel
        ArticulationDrive drive = wheel.xDrive;
        drive.target = wheel.jointPosition[0] * Mathf.Rad2Deg;
        wheel.xDrive = drive;
    }
}
