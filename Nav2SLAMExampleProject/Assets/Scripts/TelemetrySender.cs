using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;

/// <summary>
/// Sends robot telemetry data over UDP to Kavinaya's MR Dashboard.
/// Add this to each robot (robot1, robot2).
/// </summary>
public class TelemetrySender : MonoBehaviour
{
    [Header("Network Settings")]
    [Tooltip("IP address of the Quest 3 or dashboard PC")]
    public string targetIP = "127.0.0.1";
    
    [Tooltip("UDP port to send data on")]
    public int targetPort = 5005;
    
    [Header("Robot Identity")]
    public string robotId = "Robot_1";
    
    [Header("Data Source")]
    [Tooltip("Transform to track for position/rotation")]
    public Transform robotTransform;
    
    [Tooltip("Optional: Coverage agent for task progress")]
    public TurtlebotCoverageAgent coverageAgent;
    
    [Header("Settings")]
    [Tooltip("How often to send updates (times per second)")]
    public float sendRate = 10f;
    
    [Tooltip("Enable/disable sending")]
    public bool enableSending = true;
    
    // Network
    private UdpClient udpClient;
    private IPEndPoint endPoint;
    
    // Tracking
    private float lastSendTime;
    private Vector3 lastPosition;
    private float lastTime;
    
    // Telemetry data
    private float speed;
    private float batteryPercent = 100f;
    private string status = "Idle";
    private float taskProgress = 0f;
    
    void Start()
    {
        // Auto-find transform if not assigned
        if (robotTransform == null)
            robotTransform = transform;
        
        // Auto-find coverage agent
        if (coverageAgent == null)
            coverageAgent = GetComponent<TurtlebotCoverageAgent>();
        
        // Setup UDP
        try
        {
            udpClient = new UdpClient();
            endPoint = new IPEndPoint(IPAddress.Parse(targetIP), targetPort);
            Debug.Log($"[TelemetrySender] {robotId} ready to send to {targetIP}:{targetPort}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[TelemetrySender] Failed to setup UDP: {e.Message}");
            enabled = false;
        }
        
        lastPosition = robotTransform.position;
        lastTime = Time.time;
    }
    
    void Update()
    {
        if (!enableSending || udpClient == null) return;
        
        // Calculate speed
        float dt = Time.time - lastTime;
        if (dt > 0.01f)
        {
            Vector3 velocity = (robotTransform.position - lastPosition) / dt;
            speed = velocity.magnitude;
            lastPosition = robotTransform.position;
            lastTime = Time.time;
        }
        
        // Update status
        status = speed > 0.1f ? "Exploring" : "Idle";
        
        // Get coverage progress if available
        if (coverageAgent != null && coverageAgent.coverageManager != null)
        {
            taskProgress = coverageAgent.coverageManager.GetReachableCoverageFraction() * 100f;
        }
        
        // Simulate battery drain
        batteryPercent = Mathf.Max(0f, batteryPercent - Time.deltaTime * 0.01f);
        
        // Send at specified rate
        if (Time.time - lastSendTime >= 1f / sendRate)
        {
            SendTelemetry();
            lastSendTime = Time.time;
        }
    }
    
    void SendTelemetry()
    {
        try
        {
            // Create JSON-like telemetry packet
            // Format: robotId,posX,posY,posZ,rotY,speed,battery,status,taskProgress
            string data = string.Format(
                "{0},{1:F2},{2:F2},{3:F2},{4:F1},{5:F2},{6:F1},{7},{8:F1}",
                robotId,
                robotTransform.position.x,
                robotTransform.position.y,
                robotTransform.position.z,
                robotTransform.eulerAngles.y,
                speed,
                batteryPercent,
                status,
                taskProgress
            );
            
            byte[] bytes = Encoding.UTF8.GetBytes(data);
            udpClient.Send(bytes, bytes.Length, endPoint);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[TelemetrySender] Send failed: {e.Message}");
        }
    }
    
    void OnDestroy()
    {
        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
        }
    }
    
    /// <summary>
    /// Call this to update the target IP at runtime (e.g., Quest 3's IP)
    /// </summary>
    public void SetTargetIP(string ip, int port = 5005)
    {
        targetIP = ip;
        targetPort = port;
        endPoint = new IPEndPoint(IPAddress.Parse(targetIP), targetPort);
        Debug.Log($"[TelemetrySender] Target updated to {targetIP}:{targetPort}");
    }
}


