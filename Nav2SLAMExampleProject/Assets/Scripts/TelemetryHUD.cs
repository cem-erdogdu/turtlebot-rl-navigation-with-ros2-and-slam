using UnityEngine;
using TMPro;  // add a Text - TextMeshProUGUI in the Canvas

public class TelemetryHUD : MonoBehaviour
{
    public TelemetryPublisher source;       // drag the robot's publisher here
    public TextMeshProUGUI label;
    public string title = "robot1";

    void Update()
    {
        if (source == null || label == null) return;

        // Read values from the publisher
        var p = source.baseLink != null ? source.baseLink.position : source.transform.position;
        float x = p.x, y = p.z;              // Unity's forward is +Z
        float yaw = source.LastYawRad;       // see tiny change in TelemetryPublisher below

        label.text =
            $"[{title}]\n" +
            $"pos: ({x:F2}, {y:F2})  yaw: {yaw:F2} rad\n" +
            $"lin: {source.LinSpeed:F2} m/s  ang: {source.AngSpeed:F2} rad/s\n" +
            $"battery: {source.Battery:F0}%  mode: {source.Mode}";
    }
}
