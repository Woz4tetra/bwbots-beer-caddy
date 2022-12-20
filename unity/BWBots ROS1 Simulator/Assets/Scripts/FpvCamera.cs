using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum ViewMode {
    FOLLOW_ROBOT,
    FREE_CAM
}

public class FpvCamera : MonoBehaviour
{
    /// <summary>
    ///     Normal speed of camera movement.
    /// </summary>
    [SerializeField]
    float m_MovementSpeed = 5f;

    /// <summary>
    ///     Speed of camera movement when shift is held down,
    /// </summary>
    [SerializeField]
    float m_FastMovementSpeed = 10f;

    /// <summary>
    ///     Sensitivity for free look.
    /// </summary>
    [SerializeField]
    float m_FreeLookSensitivity = 3f;

    /// <summary>
    ///     Amount to zoom the camera when using the mouse wheel.
    /// </summary>
    [SerializeField]
    float m_ZoomSensitivity = 10f;

    /// <summary>
    ///     Amount to zoom the camera when using the mouse wheel (fast mode).
    /// </summary>
    [SerializeField]
    float m_FastZoomSensitivity = 50f;

    /// <summary>
    ///     Set to true when free looking (on right mouse button).
    /// </summary>
    bool m_Looking;

    bool m_FocusButtonPressed = false;

    /// <summary>
    ///     Movement acceleration
    /// </summary>
    [SerializeField]
    float m_MovementAcceleration = 10f;

    /// <summary>
    ///     Movement deceleration
    /// </summary>
    [SerializeField]
    float m_MovementDeceleration = -10f;
    
    public Component followObject;
    private Vector3 cameraPositionOffset;
    private Quaternion cameraRotationOffset;
    public float smoothSpeed = 1.0f;

    public BwbotsSimulatedChassis wheelController;
    public float forwardSpeed = 1.0f;
    public float lateralSpeed = 0.4f;
    public float angularSpeed = 3.14f;
    private float targetLinearSpeed;
    private float targetLateralSpeed;
    private float targetAngularSpeed;

    private SlewLimiter m_MovementLimiter;

    private Vector3 m_prevMovementVector = Vector3.zero;

    private ViewMode m_viewMode = ViewMode.FOLLOW_ROBOT;

    void Start()
    {
        m_MovementLimiter = new SlewLimiter(m_MovementDeceleration, m_MovementAcceleration, 0.0f);
        cameraPositionOffset = this.transform.position;
        cameraRotationOffset = this.transform.rotation;
        StopLooking();
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.V)) {
            if (m_viewMode == ViewMode.FREE_CAM) {
                m_viewMode = ViewMode.FOLLOW_ROBOT;
                Quaternion inverseRotation = Quaternion.Inverse(followObject.transform.rotation);
                cameraPositionOffset = inverseRotation * (this.transform.position - followObject.transform.position);
                cameraRotationOffset = inverseRotation * this.transform.rotation;
            }
            else {
                m_viewMode = ViewMode.FREE_CAM;
            }
        }
        
        if (m_viewMode == ViewMode.FREE_CAM) {
            FreeCamUpdate();
        }
        else if (m_viewMode == ViewMode.FOLLOW_ROBOT) {
            FollowRobotUpdate();
        }
    }

    void FollowRobotUpdate() {
        Vector3 desiredPosition = followObject.transform.position + followObject.transform.rotation * cameraPositionOffset;
        Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);
        transform.position = smoothedPosition;

        Quaternion desiredRotation = followObject.transform.rotation * cameraRotationOffset;
        Quaternion smoothedRotation = Quaternion.Lerp(transform.rotation, desiredRotation, smoothSpeed);
        transform.rotation = smoothedRotation;

        targetLinearSpeed = Input.GetAxisRaw("Vertical") * forwardSpeed;
        targetLateralSpeed = 0.0f;
        if (Input.GetKey(KeyCode.Q)) {
            targetLateralSpeed = -lateralSpeed;
            if (targetLinearSpeed == 0.0f) {
                targetLinearSpeed = forwardSpeed;
            }
        }
        if (Input.GetKey(KeyCode.E)) {
            targetLateralSpeed = lateralSpeed;
            if (targetLinearSpeed == 0.0f) {
                targetLinearSpeed = forwardSpeed;
            }
        }
        targetAngularSpeed = Input.GetAxisRaw("Horizontal") * angularSpeed;
    }

    void FixedUpdate()
    {
        wheelController.setRobotVelocity(targetLinearSpeed, targetLateralSpeed, targetAngularSpeed);
    }

    void FreeCamUpdate() {
        var fastMode = Input.GetKey(KeyCode.LeftControl);
        float movementForward = 0.0f;
        float movementRight = 0.0f;
        float movementUp = 0.0f;
        float movementSpeed = fastMode ? m_FastMovementSpeed : m_MovementSpeed;

        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
            movementRight -= movementSpeed;

        if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
            movementRight += movementSpeed;

        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
            movementForward += movementSpeed;

        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
            movementForward -= movementSpeed;

        if (Input.GetKey(KeyCode.Q) || Input.GetKey(KeyCode.LeftShift))
            movementUp -= movementSpeed;

        if (Input.GetKey(KeyCode.E) || Input.GetKey(KeyCode.Space))
            movementUp += movementSpeed;
        

        Vector3 relativeMovementVector = new Vector3(movementRight, movementUp, movementForward);
        Quaternion cameraRotation = Quaternion.Euler(0.0f, transform.localEulerAngles.y, 0.0f);
        Vector3 absoluteMovementVector = cameraRotation * relativeMovementVector;
        float movementDeltaRaw = Vector3.Distance(Vector3.zero, absoluteMovementVector);
        if (Math.Abs(movementDeltaRaw) < 1E-3) {
            absoluteMovementVector = m_prevMovementVector;
        }
        else {
            m_prevMovementVector = absoluteMovementVector;
        }
        movementDeltaRaw = SlewLimiter.clamp(movementDeltaRaw, -movementSpeed, movementSpeed);
        float movementDelta = (float)m_MovementLimiter.calculate(movementDeltaRaw);

        Vector3 movementVector = Vector3.MoveTowards(Vector3.zero, absoluteMovementVector, movementDelta);

        transform.position = transform.position + movementVector * Time.deltaTime;

        if (m_Looking)
        {
            var newRotationX = transform.localEulerAngles.y + Input.GetAxis("Mouse X") * m_FreeLookSensitivity;
            var newRotationY = transform.localEulerAngles.x - Input.GetAxis("Mouse Y") * m_FreeLookSensitivity;
            transform.localEulerAngles = new Vector3(newRotationY, newRotationX, 0f);
        }

        var axis = Input.GetAxis("Mouse ScrollWheel");
        if (axis != 0)
        {
            var zoomSensitivity = fastMode ? m_FastZoomSensitivity : m_ZoomSensitivity;
            transform.position = transform.position + transform.forward * axis * zoomSensitivity;
        }

        bool focusButtonPressed;
        if (m_Looking) {
            focusButtonPressed = Input.GetKeyDown(KeyCode.Escape);
        }
        else {
            focusButtonPressed = Input.GetKeyDown(KeyCode.Escape) || Input.GetKeyDown(KeyCode.Mouse0);
        }
        if (focusButtonPressed != m_FocusButtonPressed) {
            m_FocusButtonPressed = focusButtonPressed;
            if (!m_FocusButtonPressed) {
                if (m_Looking) {
                    StopLooking();
                }
                else {
                    StartLooking();
                }
            }
        }
    }

    void OnDisable()
    {
        StopLooking();
    }

    /// <summary>
    ///     Enable free looking.
    /// </summary>
    void StartLooking()
    {
        m_Looking = true;
        Cursor.visible = false;
        Cursor.lockState = CursorLockMode.Locked;
    }

    /// <summary>
    ///     Disable free looking.
    /// </summary>
    void StopLooking()
    {
        m_Looking = false;
        Cursor.visible = true;
        Cursor.lockState = CursorLockMode.None;
    }
}
