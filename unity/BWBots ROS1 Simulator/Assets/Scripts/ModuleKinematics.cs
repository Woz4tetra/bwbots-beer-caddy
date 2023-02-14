
using System;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.BwInterfaces;

class ModuleKinematics
{
    private String name;
    private ArticulationBody wheelBody;
    private ArticulationBody moduleBody;
    private GameObject wheel;
    private GameObject module;

    private double min_angle;
    private double max_angle;
    private double wheel_radius;
    private double min_radius_of_curvature;
    private double x_location;
    private double y_location;
    private double armature_length;
    private double prev_azimuth;
    private double max_wheel_speed;
    private BwDriveModuleMsg module_msg = new BwDriveModuleMsg();
    private bool useGroundTruth = false;

    public ModuleKinematics(
            String name,
            int index,
            GameObject wheel,
            GameObject module,
            double min_angle,
            double max_angle,
            double wheel_radius,
            double min_radius_of_curvature,
            double x_location,
            double y_location,
            double armature_length,
            double max_wheel_speed)
    {
        this.name = name;

        this.wheel = wheel;
        this.module = module;
        this.wheelBody = wheel.GetComponent<ArticulationBody>();
        this.moduleBody = module.GetComponent<ArticulationBody>();

        this.min_angle = min_angle;
        this.max_angle = max_angle;
        this.wheel_radius = wheel_radius;
        this.min_radius_of_curvature = min_radius_of_curvature;
        this.x_location = x_location;
        this.y_location = y_location;
        this.armature_length = armature_length;
        this.max_wheel_speed = max_wheel_speed;
        prev_azimuth = 0.0;

        ArticulationDrive wheelDrive = this.wheelBody.xDrive;
        wheelDrive.stiffness = 10000.0f;
        wheelDrive.damping = 0.0f;
        wheelDrive.forceLimit = 1000000.0f;
        this.wheelBody.xDrive = wheelDrive;

        this.wheelBody.linearDamping = 1.0f;
        this.wheelBody.angularDamping = 1.0f;
        this.wheelBody.jointFriction = 0.1f;
        this.wheelBody.inertiaTensor = new Vector3(6.16E-05f, 1.20E-04f, 6.15E-05f);
        this.wheelBody.mass = 0.048f;

        ArticulationDrive moduleDrive = this.moduleBody.xDrive;
        moduleDrive.stiffness = 500.0f;
        moduleDrive.damping = 10.0f;
        moduleDrive.forceLimit = 10000.0f;
        this.moduleBody.xDrive = moduleDrive;

        this.moduleBody.linearDamping = 1.0f;
        this.moduleBody.angularDamping = 1.0f;
        this.moduleBody.jointFriction = 1.0f;
        this.moduleBody.mass = 1.0f;
        this.moduleBody.inertiaTensor = new Vector3(1.59E-03f, 1.86E-04f, 1.57E-03f);

        module_msg.module_index = (index + 1).ToString();
    }

    public void setUseGroundTruth(bool useGroundTruth)
    {
        this.useGroundTruth = useGroundTruth;
    }

    private (double, double) ComputeState(double vx, double vy, double vt, double dt, double prev_azimuth)
    {
        double azimuth = prev_azimuth;
        double wheel_velocity = 0.0;

        double theta_mag = vt * dt;
        double v_mag = Math.Sqrt(vx * vx + vy * vy);
        double module_vx, module_vy;
        double radius_of_curvature;
        if (theta_mag == 0.0)
        {
            radius_of_curvature = 0.0;
        }
        else
        {
            radius_of_curvature = Math.Sign(vx) * (v_mag * dt) / Math.Tan(theta_mag);
        }

        if (theta_mag == 0.0 || Double.IsNaN(radius_of_curvature) || Double.IsInfinity(radius_of_curvature))
        {
            // Strafe regime
            module_vx = vx;
            module_vy = vy;
            if (Math.Abs(v_mag) > 0.0)
            {
                // set azimuth if v_mag is > 0.0. Otherwise, use the previous angle
                azimuth = Math.Atan2(module_vy, module_vx);
            }
            wheel_velocity = v_mag;
        }
        else if (Math.Abs(radius_of_curvature) < min_radius_of_curvature)
        {
            // Rotate in place regime
            module_vx = vt * -y_location;
            module_vy = vt * x_location;
            azimuth = Math.Atan2(module_vy, module_vx);
            wheel_velocity = Math.Sqrt(module_vx * module_vx + module_vy * module_vy);
        }
        else
        {
            // Ackermann + strafe regime
            double module_angle = Math.Atan2(x_location, radius_of_curvature - y_location);
            double module_radc = x_location / Math.Sin(module_angle) - armature_length;
            double module_hypo = Math.Sign(vx) * module_radc / radius_of_curvature;

            module_vx = module_hypo * Math.Cos(module_angle);
            module_vy = module_hypo * Math.Sin(module_angle);
            azimuth = Math.Atan2(module_vy, module_vx);
            wheel_velocity = v_mag;
        }
        return (azimuth, wheel_velocity);
    }

    public void set(double vx, double vy, double vt, double dt)
    {
        if (useGroundTruth)
        {
            return;
        }
        (double azimuth, double wheel_velocity) = ComputeState(vx, vy, vt, dt, prev_azimuth);
        prev_azimuth = azimuth;

        azimuth = WrapAngle(azimuth);
        if (azimuth < min_angle || azimuth > max_angle)
        {
            azimuth = WrapAngle(azimuth + Math.PI);
            wheel_velocity = -wheel_velocity;
        }

        double azimuthVelocity = getAzimuthVelocity();
        if (Math.Abs(azimuthVelocity) > 0.001)
        {
            double delta = azimuthVelocity * wheel_radius;
            wheel_velocity += delta;
        }

        // Debug.Log(String.Format("name: {0}, azimuth: {1}, wheel_velocity: {2}", name, azimuth, wheel_velocity));
        setModuleAzimuth(azimuth);
        setWheelVelocity(wheel_velocity);
    }

    public static double WrapAngle(double angle)
    {
        // wrap to -pi..pi
        angle = angle % (2.0 * Math.PI);
        if (angle >= Math.PI)
        {
            angle -= 2.0 * Math.PI;
        }
        if (angle < -Math.PI)
        {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    public void setWheelVelocity(double groundVelocity)
    {
        if (Math.Abs(groundVelocity) > max_wheel_speed)
        {
            groundVelocity = Math.Sign(groundVelocity) * max_wheel_speed;
        }
        double angularVelocity = -groundVelocity / wheel_radius;
        double deltaAngle = angularVelocity * Time.fixedDeltaTime;
        ArticulationDrive drive = wheelBody.xDrive;
        drive.target = drive.target + (float)(deltaAngle * Mathf.Rad2Deg);
        wheelBody.xDrive = drive;
    }

    public void setModuleAzimuth(double azimuth)
    {
        azimuth = WrapAngle(azimuth);
        if (azimuth < min_angle)
        {
            azimuth = min_angle;
        }
        if (azimuth > max_angle)
        {
            azimuth = max_angle;
        }
        azimuth *= -1.0;
        ArticulationDrive drive = moduleBody.xDrive;
        drive.target = (float)azimuth * Mathf.Rad2Deg;
        moduleBody.xDrive = drive;
    }

    public double getAzimuth()
    {
        List<float> states = new List<float>();
        moduleBody.GetJointPositions(states);

        List<int> indices = new List<int>();
        moduleBody.GetDofStartIndices(indices);

        if (moduleBody.index >= indices.Count)
        {
            Debug.LogWarning("Failed to get azimuth! Not enough joints.");
            return 0.0f;
        }
        int index = indices[moduleBody.index];
        if (index >= states.Count)
        {
            Debug.LogWarning("Failed to get azimuth! Index out of range.");
            return 0.0f;
        }
        double azimuth = (double)(states[index]);
        azimuth *= -1.0;
        azimuth = WrapAngle(azimuth);

        module_msg.azimuth_position = azimuth;
        if (x_location < 0.0)
        {
            module_msg.azimuth_position = WrapAngle(module_msg.azimuth_position + Math.PI);
        }
        return azimuth;
    }

    public double getAzimuthVelocity()
    {
        List<float> states = new List<float>();
        moduleBody.GetJointVelocities(states);

        List<int> indices = new List<int>();
        moduleBody.GetDofStartIndices(indices);

        if (moduleBody.index >= indices.Count)
        {
            Debug.LogWarning("Failed to get azimuth velocity! Not enough joints.");
            return 0.0f;
        }
        int index = indices[moduleBody.index];
        if (index >= states.Count)
        {
            Debug.LogWarning("Failed to get azimuth velocity! Index out of range.");
            return 0.0f;
        }
        double azimuthVelocity = (double)(states[index]);
        azimuthVelocity *= -1.0;
        azimuthVelocity = WrapAngle(azimuthVelocity);
        return azimuthVelocity;
    }

    public double getWheelVelocity()
    {
        List<float> states = new List<float>();
        wheelBody.GetJointVelocities(states);

        List<int> indices = new List<int>();
        wheelBody.GetDofStartIndices(indices);

        if (wheelBody.index >= indices.Count)
        {
            Debug.LogWarning("Failed to get wheel velocity! Not enough joints.");
            return 0.0f;
        }
        int index = indices[wheelBody.index];
        if (index >= states.Count)
        {
            Debug.LogWarning("Failed to get wheel velocity! Index out of range.");
            return 0.0f;
        }
        double angularVelocity = (double)(states[index]);  // rad/s
        angularVelocity = -angularVelocity * wheel_radius;
        module_msg.wheel_velocity = angularVelocity;
        return angularVelocity;
    }

    public double getXLocation()
    {
        return x_location;
    }

    public double getYLocation()
    {
        return y_location;
    }

    public BwDriveModuleMsg getMessage()
    {
        return module_msg;
    }
}
