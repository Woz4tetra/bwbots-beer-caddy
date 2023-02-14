using System;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
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
    bool looking;

    bool focusButtonPressed = false;

    /// <summary>
    ///     Movement acceleration
    /// </summary>
    [SerializeField]
    float movementAcceleration = 10f;

    /// <summary>
    ///     Movement deceleration
    /// </summary>
    [SerializeField]
    float movementDeceleration = -10f;
    
    public Component followObject;
    public Vector3 defaultCameraPositionOffset = new Vector3(0.0f, 0.35f, -0.7f);
    public Quaternion defaultCameraRotationOffset = Quaternion.Euler(0.0f, 0.0f, 0.0f);
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

    private SlewLimiter movementLimiter;

    private Vector3 prevMovementVector = Vector3.zero;

    public ViewMode startingMode = ViewMode.FOLLOW_ROBOT;
    public bool useDefaultOffsetsOnFollow = true;
    public bool useDefaultOffsetsOnStartup = true;
    private ViewMode viewMode = ViewMode.FOLLOW_ROBOT;
    private Vector3 lookingRotation = Vector3.zero;
    private bool mouseWasLocked = false;
    private int mouseWasLockedCounter = 0;

    private GUIStyle robotStatusLabelStyle = new GUIStyle();
    private int labelBorderSize;
    private float prevNonZeroRobotCommand = 0.0f;
    private float robotCommandTimeout = 0.5f;


    private Pose initialRobotPose = Pose.identity;


    void Start()
    {
        this.robotStatusLabelStyle.fontSize = 20;
        this.robotStatusLabelStyle.fontStyle = FontStyle.Bold;
        this.robotStatusLabelStyle.normal.textColor = Color.white;

        labelBorderSize = 5;
        movementLimiter = new SlewLimiter(movementDeceleration, movementAcceleration, 0.0f);

        resetLookingRotation();
        setMode(startingMode);
        if (useDefaultOffsetsOnStartup) {
            resetCameraRelativeOffsetToDefault();
        }
    }

    void resetCameraRelativeOffset() {
        if (useDefaultOffsetsOnFollow) {
            resetCameraRelativeOffsetToDefault();
        }
        else {
            Quaternion inverseRotation = Quaternion.Inverse(followObject.transform.rotation);
            cameraPositionOffset = inverseRotation * (this.transform.position - followObject.transform.position);
            cameraRotationOffset = inverseRotation * this.transform.rotation;
        }
    }

    void resetCameraRelativeOffsetToDefault() {
        cameraPositionOffset = defaultCameraPositionOffset;
        cameraRotationOffset = defaultCameraRotationOffset;
    }

    void resetLookingRotation() {
        lookingRotation.x = transform.localEulerAngles.x;
        lookingRotation.y = transform.localEulerAngles.y;
    }

    private void setMode(ViewMode mode) {
        if (mode == ViewMode.FOLLOW_ROBOT) {
            resetCameraRelativeOffset();
            StopLooking();
        }
        viewMode = mode;
        Debug.Log($"Set mode to {viewMode}");
    }

    void Update()
    {
        if (initialRobotPose.Equals(Pose.identity) && Time.realtimeSinceStartup < 30.0) {
            initialRobotPose = wheelController.GetGlobalPose();
        }
        if (Input.GetKeyDown(KeyCode.V)) {
            ViewMode mode;
            if (viewMode == ViewMode.FREE_CAM) {
                mode = ViewMode.FOLLOW_ROBOT;
            }
            else {
                mode = ViewMode.FREE_CAM;
            }
            setMode(mode);
        }
        
        if (viewMode == ViewMode.FREE_CAM) {
            FreeCamUpdate();
        }
        else if (viewMode == ViewMode.FOLLOW_ROBOT) {
            FollowRobotUpdate();
        }

        if (Input.GetKeyDown(KeyCode.C)) {
            wheelController.setMotorEnable(!wheelController.getMotorEnable());
        }
        if (Input.GetKeyDown(KeyCode.B)) {
            wheelController.setUseGroundTruth(!wheelController.getUseGroundTruth());
        }
        if (Input.GetKeyDown(KeyCode.R)) {
            initialRobotPose = wheelController.GetGlobalPose();
        }
    }

    void OnGUI()
    {
        Pose pose = wheelController.GetRelativePose(initialRobotPose);
        Vector3 position = pose.position;
        Quaternion rotation = pose.rotation;
        float angle = 360.0f - rotation.eulerAngles.y;
        if (angle > 180.0f) {
            angle -= 360.0f;
        }
        float angleRad = Mathf.Deg2Rad * angle;
        string text = $"X = {position.z:n4}\n" + 
            $"Y = {-position.x:n4}\n" + 
            $"Angle = {angleRad:n4}\n" +
            "Robot is " + (wheelController.getMotorEnable() ? "enabled" : "disabled") +
            "\nGround truth is " + (wheelController.getUseGroundTruth() ? "enabled" : "disabled");
        Vector2 size = this.robotStatusLabelStyle.CalcSize(new GUIContent(text));
        
        Rect boxRect = new Rect(Screen.width - size.x - 2 * labelBorderSize, 0, size.x + 2 * labelBorderSize, size.y + 2 * labelBorderSize);
        Rect labelRect = new Rect(boxRect.x + labelBorderSize, boxRect.y + labelBorderSize, size.x, size.y);
        GUI.Box(boxRect, GUIContent.none);
        GUI.Label(labelRect, text, this.robotStatusLabelStyle);
    }

    void FollowRobotUpdate() {
        Vector3 desiredPosition = followObject.transform.position + followObject.transform.rotation * cameraPositionOffset;
        Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);
        transform.position = smoothedPosition;

        Quaternion desiredRotation = followObject.transform.rotation * cameraRotationOffset;
        Quaternion smoothedRotation = Quaternion.Lerp(transform.rotation, desiredRotation, smoothSpeed);
        transform.rotation = smoothedRotation;

        targetAngularSpeed = 0.0f;
        targetLinearSpeed = 0.0f;
        targetLateralSpeed = 0.0f;
        if (Input.GetKey(KeyCode.A)) {
            targetAngularSpeed += angularSpeed;
        }

        if (Input.GetKey(KeyCode.D)) {
            targetAngularSpeed += -angularSpeed;
        }

        if (Input.GetKey(KeyCode.W)) {
            targetLinearSpeed += forwardSpeed;
        }

        if (Input.GetKey(KeyCode.S)) {
            targetLinearSpeed += -forwardSpeed;
        }

        if (Input.GetKey(KeyCode.Q)) {
            targetLateralSpeed += lateralSpeed;
        }

        if (Input.GetKey(KeyCode.E)) {
            targetLateralSpeed += -lateralSpeed;
        }

        if (Input.GetKeyDown(KeyCode.R)) {
            resetCameraRelativeOffsetToDefault();
        }
    }

    void FixedUpdate()
    {
        if (wheelController.getMotorEnable()) {
            sendRobotCommand(targetLinearSpeed, targetLateralSpeed, targetAngularSpeed);
        }
    }

    private void sendRobotCommand(float vx, float vy, float vt) {
        TwistMsg msg = new TwistMsg {
            linear = {
                x = vx,
                y = vy
            },
            angular = {
                z = vt
            }
        };
        if (msg.linear.x == 0.0 && msg.linear.y == 0.0 && msg.angular.z == 0.0) {
            if (prevNonZeroRobotCommand == 0.0f) {
                prevNonZeroRobotCommand = Time.realtimeSinceStartup;
            }
            if (Time.realtimeSinceStartup - prevNonZeroRobotCommand > robotCommandTimeout) {
                return;
            }
        }
        else {
            prevNonZeroRobotCommand = 0.0f;
        }
        wheelController.setTwistCommand(msg, 0, 0.1f);
    }

    void FreeCamUpdate() {
        var fastMode = Input.GetKey(KeyCode.LeftControl);
        float movementForward = 0.0f;
        float movementRight = 0.0f;
        float movementUp = 0.0f;
        float movementSpeed = fastMode ? m_FastMovementSpeed : m_MovementSpeed;

        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow)) {
            movementRight -= movementSpeed;
        }

        if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow)) {
            movementRight += movementSpeed;
        }

        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow)) {
            movementForward += movementSpeed;
        }

        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) {
            movementForward -= movementSpeed;
        }

        if (Input.GetKey(KeyCode.Q) || Input.GetKey(KeyCode.LeftShift)) {
            movementUp -= movementSpeed;
        }

        if (Input.GetKey(KeyCode.E) || Input.GetKey(KeyCode.Space)) {
            movementUp += movementSpeed;
        }
        

        Vector3 relativeMovementVector = new Vector3(movementRight, movementUp, movementForward);
        Quaternion cameraRotation = Quaternion.Euler(0.0f, transform.localEulerAngles.y, 0.0f);
        Vector3 absoluteMovementVector = cameraRotation * relativeMovementVector;
        float movementDeltaRaw = Vector3.Distance(Vector3.zero, absoluteMovementVector);
        if (Math.Abs(movementDeltaRaw) < 1E-3) {
            absoluteMovementVector = prevMovementVector;
        }
        else {
            prevMovementVector = absoluteMovementVector / movementDeltaRaw;
        }
        movementDeltaRaw = SlewLimiter.clamp(movementDeltaRaw, -movementSpeed, movementSpeed);
        float movementDelta = (float)movementLimiter.calculate(movementDeltaRaw);

        Vector3 movementVector = absoluteMovementVector * movementDelta;

        transform.position = transform.position + movementVector * Time.deltaTime;

        bool focusButtonPressed;
        if (looking) {
            focusButtonPressed = Input.GetKeyDown(KeyCode.Escape);
        }
        else {
            focusButtonPressed = Input.GetKeyDown(KeyCode.Mouse0) && IsMouseOverGameWindow(0.0f);
        }
        if (focusButtonPressed != this.focusButtonPressed) {
            this.focusButtonPressed = focusButtonPressed;
            if (!this.focusButtonPressed) {
                if (looking) {
                    StopLooking();
                }
                else {
                    StartLooking();
                }
            }
            return;
        }

        if (looking)
        {
            float deltaX = getMouseDeltaX();
            float deltaY = getMouseDeltaY();
            // if (deltaX != 0.0f || deltaY != 0.0f) {
            //     Debug.Log($"deltaX: {deltaX}, deltaY: {deltaY}");
            // }
            if (mouseWasLocked) {
                if (mouseWasLockedCounter == 0 && deltaX == 0.0f && deltaY == 0.0f) {
                    return;
                }
                mouseWasLockedCounter++;
                if (mouseWasLockedCounter > 20 && deltaX == 0.0f && deltaY == 0.0f) {
                    Debug.Log($"cursor is settled");
                    mouseWasLocked = false;
                }
                else {
                    return;
                }
            }

            lookingRotation.y += deltaX * m_FreeLookSensitivity;
            lookingRotation.x += -deltaY * m_FreeLookSensitivity;

            transform.localEulerAngles = lookingRotation;
        }

        var axis = Input.GetAxis("Mouse ScrollWheel");
        if (axis != 0)
        {
            var zoomSensitivity = fastMode ? m_FastZoomSensitivity : m_ZoomSensitivity;
            transform.position = transform.position + transform.forward * axis * zoomSensitivity;
        }

    }
    bool IsMouseOverGameWindow(float tolerance) {
        return !(tolerance > Input.mousePosition.x || 
            tolerance > Input.mousePosition.y || 
            (Screen.width - tolerance) < Input.mousePosition.x || 
            (Screen.height - tolerance) < Input.mousePosition.y
        );
    }
    private float getMouseDeltaX() {
        return Input.GetAxisRaw("Mouse X");
    }
    private float getMouseDeltaY() {
        return Input.GetAxisRaw("Mouse Y");
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
        looking = true;
        Cursor.visible = false;
        // Cursor.visible = true;
        // Cursor.lockState = CursorLockMode.Confined;
        Cursor.lockState = CursorLockMode.Locked;
        Debug.Log("Locking cursor");
        resetLookingRotation();
        mouseWasLocked = true;
        mouseWasLockedCounter = 0;
    }

    /// <summary>
    ///     Disable free looking.
    /// </summary>
    void StopLooking()
    {
        looking = false;
        Cursor.visible = true;
        Cursor.lockState = CursorLockMode.None;
    }
}
