using UnityEngine;
using UnityEngine.UI;

public class TelemetryBatteryBar : MonoBehaviour
{
    public TelemetryPublisher source; // drag robot's TelemetryPublisher here
    public Image fill;                // drag Battery_fill_robot1 here

    void Update()
    {
        if (!source || !fill) return;
        float t = Mathf.Clamp01(source.Battery / 100f);
        fill.fillAmount = t;
        // optional color shift green→red
        fill.color = Color.Lerp(Color.red, Color.green, t);
    }
}
