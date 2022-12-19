using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowObjectCamera : MonoBehaviour
{
    public Component followObject;
    private Vector3 cameraPositionOffset;
    private Quaternion cameraRotationOffset;
    void Start()
    {
        cameraPositionOffset = this.transform.position;
        cameraRotationOffset = this.transform.rotation;
    }

    void Update()
    {
        transform.position = cameraPositionOffset + followObject.transform.position;
    }

    void OnDisable()
    {
        
    }
}
