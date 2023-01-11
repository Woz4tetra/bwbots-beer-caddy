using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using System;

public class LaserScanSensor : MonoBehaviour
{
    public bool debug = false;
    public double ScanDelay = 0.1;
    public float RangeMetersMin = 0;
    public float RangeMetersMax = 1000;
    public float ScanAngleStartDegrees = -45;
    public float ScanAngleEndDegrees = 45;
    // Change the scan start and end by this amount after every publish
    public float ScanOffsetAfterPublish = 0f;
    public int NumMeasurementsPerScan = 10;
    public float TimeBetweenMeasurementsSeconds = 0.01f;
    public string FrameId = "laser";
    public float noiseMagnitude = 0.05f;

    float m_CurrentScanAngleStart;
    float m_CurrentScanAngleEnd;
    double m_TimeNextScanSeconds = -1;
    int m_NumMeasurementsTaken;
    List<float> ranges = new List<float>();
    List<float> intensities = new List<float>();

    bool isScanning = false;
    double m_TimeLastScanBeganSeconds = -1;

    private LaserScanMsg latestScanMsg;

    [SerializeField] private double PublishDelay;
    [SerializeField] private string topic;
    private double _prevScanTime;
    private uint m_messageCount = 0;
    
    private ROSConnection _ros;


    protected virtual void Start()
    {
        m_CurrentScanAngleStart = ScanAngleStartDegrees;
        m_CurrentScanAngleEnd = ScanAngleEndDegrees;

        m_TimeNextScanSeconds = Time.realtimeSinceStartup + ScanDelay;

        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<LaserScanMsg>(topic);
        _prevScanTime = Time.realtimeSinceStartup;
        Debug.Log($"Laser topic registered: {topic}");
    }

    void BeginScan()
    {
        isScanning = true;
        m_TimeLastScanBeganSeconds = Time.realtimeSinceStartup;
        m_TimeNextScanSeconds = m_TimeLastScanBeganSeconds + ScanDelay;
        m_NumMeasurementsTaken = 0;
    }

    private void VisualizeScan(LaserScanMsg scanMsg)
    {
        var currAngle = scanMsg.angle_min;
        foreach (var range in scanMsg.ranges.Reverse())
        {
            if (float.IsFinite(range)) {
                Vector3 direction = Quaternion.Euler(0, currAngle * Mathf.Rad2Deg, 0) * transform.forward;
                Debug.DrawRay(transform.position, direction * range, new Color(1.0f, 0.0f, 0.0f, 0.5f), duration:(float)(ScanDelay));
            }
            currAngle += scanMsg.angle_increment;
        }
    }

    public void EndScan()
    {
        if (ranges.Count == 0)
        {
            Debug.LogWarning($"Took {m_NumMeasurementsTaken} measurements but found no valid ranges");
        }
        else if (ranges.Count != m_NumMeasurementsTaken || ranges.Count != NumMeasurementsPerScan)
        {
            Debug.LogWarning($"Expected {NumMeasurementsPerScan} measurements. Actually took {m_NumMeasurementsTaken}" +
                             $"and recorded {ranges.Count} ranges.");
        }
        
        var angleStartRos = m_CurrentScanAngleStart * Mathf.Deg2Rad;
        var angleEndRos = m_CurrentScanAngleEnd * Mathf.Deg2Rad;

        // Invert the angle ranges when going from Unity to ROS
        // var angleStartRos = -m_CurrentScanAngleStart * Mathf.Deg2Rad;
        // var angleEndRos = -m_CurrentScanAngleEnd * Mathf.Deg2Rad;
        // if (angleStartRos > angleEndRos)
        // {
            // Debug.LogWarning("LaserScan was performed in a clockwise direction but ROS expects a counter-clockwise scan, flipping the ranges...");
            // var temp = angleEndRos;
            // angleEndRos = angleStartRos;
            // angleStartRos = temp;
        // }
        
        ranges.Reverse();
        intensities.Reverse();

        var msg = new LaserScanMsg
        {
            header = new HeaderMsg
            {
                frame_id = FrameId,
                seq = m_messageCount,
                stamp = RosUtil.GetTimeMsg()
            },
            range_min = RangeMetersMin,
            range_max = RangeMetersMax,
            angle_min = angleStartRos,
            angle_max = angleEndRos,
            angle_increment = (angleEndRos - angleStartRos) / NumMeasurementsPerScan,
            time_increment = TimeBetweenMeasurementsSeconds,
            scan_time = (float)ScanDelay,
            intensities = intensities.ToArray(),
            ranges = ranges.ToArray(),
        };

        this.latestScanMsg = msg;

        if (this.debug)
            this.VisualizeScan(msg);

        m_NumMeasurementsTaken = 0;
        m_messageCount++;
        ranges.Clear();
        intensities.Clear();
        isScanning = false;
    }


    public LaserScanMsg GetLaserScanMsg() {
        return latestScanMsg;
    }

    public void Update()
    {
        double now = Time.realtimeSinceStartup;
        if (now - _prevScanTime > PublishDelay)
        {
            _ros.Publish(topic, GetLaserScanMsg());
            _prevScanTime = now;
        }

        if (!isScanning)
        {
            if (Time.realtimeSinceStartup < m_TimeNextScanSeconds)
            {
                return;
            }

            BeginScan();
        }

        var measurementsSoFar = TimeBetweenMeasurementsSeconds == 0 ? NumMeasurementsPerScan :
            1 + Mathf.FloorToInt((float)(Time.realtimeSinceStartup - m_TimeLastScanBeganSeconds) / TimeBetweenMeasurementsSeconds);
        if (measurementsSoFar > NumMeasurementsPerScan)
            measurementsSoFar = NumMeasurementsPerScan;

        var yawBaseDegrees = transform.rotation.eulerAngles.y;
        while (m_NumMeasurementsTaken < measurementsSoFar)
        {
            var t = m_NumMeasurementsTaken / (float)NumMeasurementsPerScan;
            var yawSensorDegrees = Mathf.Lerp(m_CurrentScanAngleStart, m_CurrentScanAngleEnd, t);
            var yawDegrees = yawBaseDegrees + yawSensorDegrees;
            var directionVector = Quaternion.Euler(0f, yawDegrees, 0f) * Vector3.forward;
            var measurementRay = new Ray(transform.position, directionVector);
            var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, RangeMetersMax);
            // Only record measurement if it's within the sensor's operating range
            if (foundValidMeasurement && hit.distance > RangeMetersMin)
            {
                float magnitude = noiseMagnitude * hit.distance / RangeMetersMax;
                float noise = UnityEngine.Random.Range(-magnitude, magnitude);
                ranges.Add(hit.distance + noise);
                intensities.Add(100.0f);
            }
            else
            {
                ranges.Add(float.PositiveInfinity);
                intensities.Add(0.0f);
            }

            // Even if Raycast didn't find a valid hit, we still count it as a measurement
            ++m_NumMeasurementsTaken;
        }
        
        if (m_NumMeasurementsTaken >= NumMeasurementsPerScan)
        {
            if (m_NumMeasurementsTaken > NumMeasurementsPerScan)
            {
                Debug.LogError($"LaserScan has {m_NumMeasurementsTaken} measurements but we expected {NumMeasurementsPerScan}");
            }
            EndScan();
        }

    }
}
