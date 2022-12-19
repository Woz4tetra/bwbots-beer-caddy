
using System;
using UnityEngine;

class ModuleKinematics
{
    private String name;
    private ArticulationBody wheel;
    private ArticulationBody module;

    private double min_angle;
    private double max_angle;
    private double min_radius_of_curvature;
    private double x_location;
    private double y_location;
    private double armature_length;
    private double azimuth_direction;
    private double wheel_direction;
    private double prev_azimuth;
    private double max_wheel_speed;

    public ModuleKinematics(
            String name,
            ArticulationBody wheel,
            ArticulationBody module,
            double min_angle,
            double max_angle,
            double min_radius_of_curvature,
            double x_location,
            double y_location,
            double armature_length,
            double max_wheel_speed,
            bool azimuth_direction,
            bool wheel_direction) {
        this.name = name;
        this.wheel = wheel;
        this.module = module;

        this.min_angle = min_angle;
        this.max_angle = max_angle;
        this.min_radius_of_curvature = min_radius_of_curvature;
        this.x_location = x_location;
        this.y_location = y_location;
        this.armature_length = armature_length;
        this.max_wheel_speed = max_wheel_speed;
        this.azimuth_direction = azimuth_direction ? 1.0 : -1.0;
        this.wheel_direction = wheel_direction ? 1.0 : -1.0;
        prev_azimuth = 0.0;

        ArticulationDrive wheelDrive = wheel.xDrive;
        wheelDrive.stiffness = 200.0f;
        wheelDrive.damping = 25.0f;
        wheelDrive.forceLimit = 3.402823e+38f;
        wheel.xDrive = wheelDrive;

        wheel.linearDamping = 0.1f;
        wheel.angularDamping = 0.1f;
        wheel.jointFriction = 0.1f;
        wheel.mass = 0.01f;

        ArticulationDrive moduleDrive = module.xDrive;
        moduleDrive.stiffness = 10000.0f;
        moduleDrive.damping = 1000.0f;
        moduleDrive.forceLimit = 3.402823e+38f;
        module.xDrive = moduleDrive;

        module.linearDamping = 1.0f;
        module.angularDamping = 1.0f;
        module.jointFriction = 1.0f;
        module.mass = 0.1f;
    }

    private (double, double) ComputeState(double vx, double vy, double vt, double dt, double prev_azimuth)
    {
        double azimuth = prev_azimuth;
        double wheel_velocity = 0.0;

        double theta_mag = vt * dt;
        double v_mag = Math.Sqrt(vx * vx + vy * vy);
        double module_vx, module_vy;
        double radius_of_curvature;
        if (theta_mag == 0.0) {
            radius_of_curvature = 0.0;
        }
        else {
            radius_of_curvature = Math.Sign(vx) * (v_mag * dt) / Math.Tan(theta_mag);
        }
        
        if (theta_mag == 0.0 || Double.IsNaN(radius_of_curvature) || Double.IsInfinity(radius_of_curvature)) {
            // Strafe regime
            module_vx = vx;
            module_vy = vy;
            if (Math.Abs(v_mag) > 0.0) {
                // set azimuth if v_mag is > 0.0. Otherwise, use the previous angle
                azimuth = Math.Atan2(module_vy, module_vx);
            }
            wheel_velocity = v_mag;
        }
        else if (Math.Abs(radius_of_curvature) < min_radius_of_curvature) {
            // Rotate in place regime
            module_vx = vt * -y_location;
            module_vy = vt * x_location;
            azimuth = Math.Atan2(module_vy, module_vx);
            wheel_velocity = Math.Sqrt(module_vx * module_vx + module_vy * module_vy);
        }
        else {
            // Ackermann + strafe regime
            double module_angle = Math.Atan2(x_location, radius_of_curvature - y_location);
            double module_radc = x_location / Math.Sin(module_angle) - armature_length;
            double module_hypo = Math.Sign(vx) * module_radc / radius_of_curvature;

            module_vx = module_hypo * Math.Cos(module_angle);
            module_vy = module_hypo * Math.Sin(module_angle);
            azimuth = Math.Atan2(module_vy, module_vx);
        }
        return (azimuth, wheel_velocity);
    }

    public void set(double vx, double vy, double vt, double dt) {
        (double azimuth, double wheel_velocity) = ComputeState(vx, vy, vt, dt, prev_azimuth);
        prev_azimuth = azimuth;

        azimuth = WrapAngle(azimuth);
        if (azimuth < min_angle || azimuth > max_angle) {
            azimuth = WrapAngle(azimuth + Math.PI);
            wheel_velocity = -wheel_velocity;
        }

        Debug.Log(String.Format("name: {0}, azimuth: {1}, wheel_velocity: {2}", name, azimuth, wheel_velocity));
        setModuleAzimuth(azimuth);
        setWheelVelocity(wheel_velocity);
    }

    private double WrapAngle(double angle)
    {
        // wrap to -pi..pi
        angle = angle % (2.0 * Math.PI);
        if (angle >= Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        if (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
    
    public void setWheelVelocity(double jointVelocity)
    {
        if (Math.Abs(jointVelocity) > max_wheel_speed) {
            jointVelocity = Math.Sign(jointVelocity) * max_wheel_speed;
        }
        jointVelocity = this.wheel_direction * jointVelocity;
        ArticulationDrive drive = wheel.xDrive;
        drive.target = drive.target + (float)jointVelocity * 1500.0f * Time.fixedDeltaTime;
        wheel.xDrive = drive;
    }

    public void setModuleAzimuth(double azimuth) {
        azimuth = this.azimuth_direction * azimuth;
        azimuth = WrapAngle(azimuth);
        if (azimuth < min_angle) {
            azimuth = min_angle;
        }
        if (azimuth > max_angle) {
            azimuth = max_angle;
        }
        ArticulationDrive drive = module.xDrive;
        drive.target = (float)azimuth * Mathf.Rad2Deg;
        module.xDrive = drive;
    }
}
