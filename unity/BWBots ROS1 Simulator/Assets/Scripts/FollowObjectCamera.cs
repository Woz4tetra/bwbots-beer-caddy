using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowObjectCamera : MonoBehaviour
{
    public Component followObject;
    private Vector3 cameraPositionOffset;
    private Quaternion cameraRotationOffset;
    public float smoothSpeed = 0.125f;
    void Start()
    {
        cameraPositionOffset = this.transform.position;
        cameraRotationOffset = this.transform.rotation;
    }

    void Update()
    {
        Vector3 desiredPosition = followObject.transform.position + followObject.transform.rotation * cameraPositionOffset;
        Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);
        transform.position = smoothedPosition;

        Quaternion desiredRotation = followObject.transform.rotation * cameraRotationOffset;
        Quaternion smoothedRotation = Quaternion.Lerp(transform.rotation, desiredRotation, smoothSpeed);
        transform.rotation = smoothedRotation;
    }

    void OnDisable()
    {
        
    }
}
