using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System;

public class TelemetryPublisher : MonoBehaviour
{
    [Header("Identity")]
    public string robotName = "robot1";
    public Transform baseLink;                    // drag base_footprint or base_link

    [Header("Optional link to controller (for mode)")]
    public RosSharp.Control.AGVController controller;

    [Header("Publish settings")]
    public float publishHz = 10f;

    [Header("Battery model (per second)")]
    public float idleDrainPerSecond = 0.001f;     // drains very slowly when stopped
    public float moveDrainBase       = 0.004f;    // base drain when moving
    public float linDrainPerMs       = 0.010f;    // extra per m/s
    public float angDrainPerRad      = 0.005f;    // extra per rad/s

    ROSConnection ros;

    // internal state for speed calc
    Vector3 lastPos;
    float   lastYawDeg;

    // ----- Public telemetry (HUD reads these) -----
    public float LinSpeed   { get; private set; }    // m/s
    public float AngSpeed   { get; private set; }    // rad/s
    public float Battery    { get; private set; } = 100f; // %
    public float LastYawRad => baseLink ? baseLink.eulerAngles.y * Mathf.Deg2Rad : 0f;

    public string Mode
        => (controller && controller.mode == RosSharp.Control.ControlMode.Keyboard) ? "keyboard" : "ros";

    [Serializable]
    class Telemetry
    {
        public double stamp;
        public float x;     // Unity X
        public float y;     // Unity Z (explicit)
        public float yaw;   // radians (from Unity yaw)
        public float lin;   // m/s
        public float ang;   // rad/s
        public float battery; // %
        public string mode;
    }

    void Start()
    {
        if (!baseLink) baseLink = transform;          // fallback

        lastPos   = baseLink.position;
        lastYawDeg = baseLink.eulerAngles.y;

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>($"/{robotName}/telemetry");
    }

    float _accum;

    void Update()
    {
        if (!baseLink) return;

        float dt = Mathf.Max(Time.unscaledDeltaTime, 1e-6f);

        // --- speeds from pose deltas ---
        Vector3 dp = baseLink.position - lastPos;
        LinSpeed = dp.magnitude / dt;

        float yawDeg  = baseLink.eulerAngles.y;
        float dYawDeg = Mathf.DeltaAngle(lastYawDeg, yawDeg);
        AngSpeed = (dYawDeg * Mathf.Deg2Rad) / dt;

        lastPos   = baseLink.position;
        lastYawDeg = yawDeg;

        // --- battery model ---
        bool moving = (LinSpeed > 0.02f) || (Mathf.Abs(AngSpeed) > 0.02f);
        float drain = moving
            ? moveDrainBase + linDrainPerMs * LinSpeed + angDrainPerRad * Mathf.Abs(AngSpeed)
            : idleDrainPerSecond;

        Battery = Mathf.Max(0f, Battery - drain * dt);

        // --- publish at fixed Hz ---
        _accum += dt;
        if (_accum >= 1f / Mathf.Max(publishHz, 0.01f))
        {
            _accum = 0f;
            Publish();
        }
    }

    void Publish()
    {
        Vector3 p = baseLink.position;

        var t = new Telemetry
        {
            stamp   = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0,
            x       = p.x,
            y       = p.z,             // Unity forward is +Z
            yaw     = LastYawRad,
            lin     = LinSpeed,
            ang     = AngSpeed,
            battery = Battery,
            mode    = Mode
        };

        string json = JsonUtility.ToJson(t);
        ros.Publish($"/{robotName}/telemetry", new StringMsg(json));
    }
}
