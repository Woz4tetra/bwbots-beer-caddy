using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
///     This script detects keyboard controls and
///     use them to control the mobile robot
/// </summary>
public class KeyboardControl : MonoBehaviour
{
    public ArticulationWheelController wheelController;

    public float speed = 1.0f;
    public float angularSpeed = 3.14f;
    private float targetLinearSpeed;
    private float targetAngularSpeed;

    void Start()
    {
    }

    void Update()
    {
        // Get key input
        targetLinearSpeed = Input.GetAxisRaw("Vertical") * speed;
        targetAngularSpeed = Input.GetAxisRaw("Horizontal") * angularSpeed;
    }

    void FixedUpdate()
    {
        wheelController.setRobotVelocity(targetLinearSpeed, 0.0f, targetAngularSpeed);
    }
}
