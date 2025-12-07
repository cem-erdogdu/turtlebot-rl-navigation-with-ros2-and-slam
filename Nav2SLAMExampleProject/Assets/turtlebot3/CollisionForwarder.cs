// ============================================================================
// CollisionForwarder.cs - Forward collision events to parent TurtlebotCoverageAgent
// ============================================================================
//
// WHY THIS IS NEEDED:
// Unity's OnCollisionEnter/Stay/Exit callbacks only fire on:
// 1. The GameObject with the Collider
// 2. The GameObject with the Rigidbody/ArticulationBody attached to that collider
//
// In URDF robots, the hierarchy is:
//   robot1 (TurtlebotCoverageAgent is here - NO COLLIDER!)
//     └── base_footprint (ArticulationBody root)
//         └── base_link (Colliders are HERE!)
//             └── Collisions (child colliders)
//
// So collision callbacks go to base_link, NOT to robot1!
// This script goes on base_link and forwards collisions up to the agent.
//
// HOW IT WORKS:
// TurtlebotCoverageAgent.Initialize() automatically finds all child Colliders
// and adds a CollisionForwarder to each one. You don't need to do anything!
//
// ============================================================================

using UnityEngine;

public class CollisionForwarder : MonoBehaviour
{
    [Header("Target Agent (Auto-set by TurtlebotCoverageAgent)")]
    [Tooltip("The TurtlebotCoverageAgent to forward collisions to")]
    public TurtlebotCoverageAgent targetAgent;
    
    [Header("Debug")]
    public bool showDebugLogs = false;  // Default false to reduce spam
    
    private void Start()
    {
        // Auto-find the agent in parent hierarchy if not already set
        if (targetAgent == null)
        {
            targetAgent = GetComponentInParent<TurtlebotCoverageAgent>();
            
            if (targetAgent == null)
            {
                Debug.LogError($"[CollisionForwarder] No TurtlebotCoverageAgent found in parent hierarchy of {gameObject.name}!");
            }
            else if (showDebugLogs)
            {
                Debug.Log($"[CollisionForwarder] Auto-found agent on {targetAgent.gameObject.name} for collider on {gameObject.name}");
            }
        }
    }
    
    private void OnCollisionEnter(Collision collision)
    {
        // Skip logging floor/ground to reduce spam
        string nameLower = collision.gameObject.name.ToLower();
        bool isFloor = nameLower.Contains("floor") || nameLower.Contains("ground") || 
                       nameLower.Contains("plane");
        
        if (showDebugLogs && !isFloor)
            Debug.Log($"[CollisionForwarder:{gameObject.name}] OnCollisionEnter with {collision.gameObject.name}");
        
        if (targetAgent != null)
        {
            targetAgent.ForwardCollision(collision.gameObject);
        }
    }
    
    private void OnCollisionStay(Collision collision)
    {
        // Only forward occasionally to avoid spam (every 10 frames)
        if (Time.frameCount % 10 == 0)
        {
            if (targetAgent != null)
            {
                targetAgent.ForwardCollision(collision.gameObject);
            }
        }
    }
}

