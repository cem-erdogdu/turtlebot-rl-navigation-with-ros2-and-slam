// ============================================================================
// TurtlebotCoverageAgent.cs - Multi-Robot Collaborative Exploration Agent
// ============================================================================
//
// PURPOSE:
// Train TWO Turtlebots to collaboratively explore a warehouse while:
// - Avoiding ALL obstacles (walls, shelves, boxes)
// - Avoiding EACH OTHER
// - Sharing a common goal (maximize total map coverage)
//
// ============================================================================
// KEY DESIGN DECISIONS:
// ============================================================================
//
// 1. SHARED REWARD: Both robots share the same coverage reward.
//    If Robot1 discovers a new area, Robot2 also benefits.
//    This encourages them to SPREAD OUT rather than follow each other.
//
// 2. OBSTACLE AVOIDANCE: Uses raycasts to "see" obstacles and penalizes:
//    - Getting too close to obstacles (soft penalty)
//    - Actually colliding (hard penalty + episode end)
//
// 3. MULTI-ROBOT AWARENESS: Each robot observes the other robot's position
//    so it can learn to avoid collisions and coordinate exploration.
//
// ============================================================================
// OBSERVATIONS (15 total):
// ============================================================================
//
// Self State (4):
//   1-2. Robot position (x, z) normalized
//   3-4. Robot forward direction (x, z)
//
// Other Robot (3):
//   5.   Relative direction to other robot (forward component)
//   6.   Relative direction to other robot (right component)
//   7.   Distance to other robot (normalized)
//
// Environment (2):
//   8.   Global coverage fraction (0-1)
//   9.   Local unexplored density around robot (0-1)
//
// Obstacle Sensing (6):
//   10-15. Raycast distances in front (6 rays, normalized 0-1)
//
// ============================================================================

using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using RosSharp.Control;
using System.Collections.Generic;

public class TurtlebotCoverageAgent : Agent
{
    // ========================================================================
    // ROBOT REFERENCES
    // ========================================================================
    
    [Header("Robot References")]
    [Tooltip("This robot's base transform (base_footprint)")]
    public Transform robotRoot;
    
    [Tooltip("Reference to AGVController")]
    public AGVController agvController;
    
    [Tooltip("The OTHER robot in the scene (for multi-agent awareness)")]
    public Transform otherRobot;
    
    [Header("Coverage System")]
    [Tooltip("Reference to shared MapCoverageManager (auto-finds if null)")]
    public MapCoverageManager coverageManager;
    
    // ========================================================================
    // MOVEMENT SETTINGS
    // ========================================================================
    
    [Header("Movement")]
    public float maxLinearSpeed = 0.5f;
    public float maxAngularSpeed = 1.0f;
    
    [Tooltip("How far robot can go from origin before episode ends. Set high for large warehouses!")]
    public float boundaryLimit = 15f;  // Increased for larger warehouses
    
    // ========================================================================
    // RAYCAST SETTINGS (Obstacle Detection)
    // ========================================================================
    
    [Header("Raycast Obstacle Detection")]
    [Tooltip("Number of rays to cast")]
    public int numRays = 6;
    
    [Tooltip("Maximum detection distance")]
    public float rayMaxDistance = 3f;
    
    [Tooltip("Angular spread of rays (degrees)")]
    public float rayAngleSpan = 150f;
    
    [Tooltip("Height offset for rays")]
    public float rayHeight = 0.15f;
    
    [Tooltip("Layers to detect as obstacles")]
    public LayerMask obstacleLayerMask = -1;  // All layers by default
    
    // ========================================================================
    // REWARD SETTINGS
    // ========================================================================
    
    [Header("Rewards - Exploration")]
    [Tooltip("Reward per new cell discovered")]
    public float newCellReward = 1.0f;
    
    [Tooltip("Bonus reward multiplied by coverage delta (shared team reward)")]
    public float coverageDeltaScale = 50f;
    
    [Tooltip("Small penalty per step (encourages efficiency)")]
    public float timeStepPenalty = 0.001f;
    
    [Header("Rewards - Movement")]
    [Tooltip("Small reward for moving forward (discourages standing still)")]
    public float forwardMovementReward = 0.001f;
    
    [Header("Rewards - Obstacle Avoidance")]
    [Tooltip("Distance threshold for 'too close' warning")]
    public float dangerDistance = 0.5f;
    
    [Tooltip("Penalty scale when too close to obstacles")]
    public float proximityPenaltyScale = 0.01f;
    
    [Tooltip("Hard penalty for collision with obstacle")]
    public float obstacleCollisionPenalty = 1.0f;
    
    [Tooltip("Hard penalty for collision with other robot")]
    public float robotCollisionPenalty = 1.5f;
    
    [Tooltip("Penalty for going out of bounds")]
    public float outOfBoundsPenalty = 1.0f;
    
    [Header("Rewards - Stuck Detection")]
    [Tooltip("Penalty when stuck")]
    public float stuckPenalty = 0.5f;
    
    [Tooltip("Movement threshold to detect if stuck (meters per step)")]
    public float stuckThreshold = 0.001f;
    
    [Tooltip("Steps of trying to move but stuck before ending episode")]
    public int stuckStepsLimit = 200;
    
    // ========================================================================
    // EPISODE SETTINGS
    // ========================================================================
    
    [Header("Episode Settings")]
    [Tooltip("End episode after this many obstacle collisions (set high for testing!)")]
    public int maxObstacleCollisions = 10;
    
    [Tooltip("End episode on robot-robot collision")]
    public bool endOnRobotCollision = false;  // False by default for easier testing
    
    [Header("Collision Detection Tags")]
    [Tooltip("Tags that count as obstacles for collision detection")]
    public string[] obstacleTags = { "Obstacle", "Shelf", "Wall", "Box" };
    
    [Tooltip("If true, only tagged objects count as obstacles. If false, everything is an obstacle.")]
    public bool useTagBasedCollision = true;
    
    // ========================================================================
    // DEBUG
    // ========================================================================
    
    [Header("Debug")]
    public bool showRaycastDebug = true;
    public bool showDebugLogs = false;
    public bool logRewardBreakdown = false;
    
    // ========================================================================
    // INTERNAL STATE
    // ========================================================================
    
    private Vector3 startPos;
    private Quaternion startRot;
    private ArticulationBody rootArticulationBody;
    
    // Collision tracking
    private int obstacleCollisionCount = 0;
    private bool robotCollisionThisStep = false;
    private bool obstacleCollisionThisStep = false;
    
    // Stuck detection
    private Vector3 lastPosition;
    private int stuckCounter = 0;
    
    // Raycast results cache
    private float[] rayDistances;
    
    // Episode stats
    private int episodeStepCount = 0;
    private float episodeTotalReward = 0f;
    
    // ========================================================================
    // INITIALIZE
    // ========================================================================
    
    public override void Initialize()
    {
        // Auto-find references
        if (robotRoot == null)
            robotRoot = transform;
        
        if (agvController == null)
            agvController = GetComponent<AGVController>();
        
        if (coverageManager == null)
            coverageManager = MapCoverageManager.Instance;
        
        // Find ArticulationBody for physics reset
        rootArticulationBody = robotRoot.GetComponentInParent<ArticulationBody>();
        if (rootArticulationBody == null)
            rootArticulationBody = robotRoot.GetComponent<ArticulationBody>();
        
        // Store start pose
        startPos = robotRoot.position;
        startRot = robotRoot.rotation;
        
        // Initialize raycast cache
        rayDistances = new float[numRays];
        
        // Auto-find other robot if not set
        if (otherRobot == null)
        {
            TurtlebotCoverageAgent[] agents = FindObjectsOfType<TurtlebotCoverageAgent>();
            foreach (var agent in agents)
            {
                if (agent != this && agent.robotRoot != null)
                {
                    otherRobot = agent.robotRoot;
                    break;
                }
            }
        }
        
        // ====================================================================
        // AUTO-SETUP COLLISION FORWARDERS
        // ====================================================================
        // Unity collision callbacks only fire on the GameObject with the Collider.
        // For URDF robots, colliders are on child objects (base_link, etc.),
        // NOT on the parent where our agent script lives.
        // So we need to add CollisionForwarder scripts to all child colliders.
        
        SetupCollisionForwarders();
        
        // Always log initialization info
        Debug.Log($"[{gameObject.name}] TurtlebotCoverageAgent initialized");
        Debug.Log($"  Start position: {startPos}");
        Debug.Log($"  Boundary limit: {boundaryLimit}");
        Debug.Log($"  Other robot: {(otherRobot != null ? otherRobot.name : "NOT FOUND")}");
        Debug.Log($"  Coverage manager: {(coverageManager != null ? "Found" : "NOT FOUND")}");
        
        // Warn if starting outside boundary
        if (Mathf.Abs(startPos.x) > boundaryLimit || Mathf.Abs(startPos.z) > boundaryLimit)
        {
            Debug.LogError($"[{gameObject.name}] ❌ CRITICAL: Robot starts OUTSIDE boundary! " +
                           $"Position=({startPos.x:F1}, {startPos.z:F1}), Limit=±{boundaryLimit}. " +
                           $"This will cause INSTANT episode reset! " +
                           $"FIX: Increase 'Boundary Limit' to {Mathf.Max(Mathf.Abs(startPos.x), Mathf.Abs(startPos.z)) + 2f:F0} or higher!");
        }
        else
        {
            Debug.Log($"  ✓ Position within boundary: ({startPos.x:F1}, {startPos.z:F1}) < ±{boundaryLimit}");
        }
    }
    
    /// <summary>
    /// Automatically adds CollisionForwarder scripts to child objects.
    /// 
    /// IMPORTANT: For ArticulationBody physics (URDF robots), collision callbacks
    /// fire on the ArticulationBody's GameObject, NOT the child with the Collider!
    /// 
    /// So we need to add forwarders to BOTH:
    /// 1. GameObjects with Colliders (for regular Rigidbody physics)
    /// 2. GameObjects with ArticulationBodies (for articulated physics)
    /// </summary>
    private void SetupCollisionForwarders()
    {
        int forwardersAdded = 0;
        HashSet<GameObject> processedObjects = new HashSet<GameObject>();
        
        // ================================================================
        // STEP 1: Add forwarders to all ArticulationBody GameObjects
        // ================================================================
        // For URDF robots, collision callbacks fire on ArticulationBody, 
        // not on the child Collider object!
        ArticulationBody[] bodies = GetComponentsInChildren<ArticulationBody>();
        foreach (ArticulationBody body in bodies)
        {
            if (body.gameObject == gameObject)
                continue;
            
            if (processedObjects.Contains(body.gameObject))
                continue;
            
            CollisionForwarder existing = body.GetComponent<CollisionForwarder>();
            if (existing == null)
            {
                CollisionForwarder forwarder = body.gameObject.AddComponent<CollisionForwarder>();
                forwarder.targetAgent = this;
                forwarder.showDebugLogs = showDebugLogs;
                forwardersAdded++;
            }
            else
            {
                existing.targetAgent = this;
            }
            
            processedObjects.Add(body.gameObject);
        }
        
        // ================================================================
        // STEP 2: Add forwarders to Collider GameObjects (as backup)
        // ================================================================
        // Some colliders might not be associated with an ArticulationBody
        Collider[] childColliders = GetComponentsInChildren<Collider>();
        foreach (Collider col in childColliders)
        {
            if (col.gameObject == gameObject)
                continue;
            
            if (col.isTrigger)
                continue;
            
            if (processedObjects.Contains(col.gameObject))
                continue;
            
            CollisionForwarder existing = col.GetComponent<CollisionForwarder>();
            if (existing == null)
            {
                CollisionForwarder forwarder = col.gameObject.AddComponent<CollisionForwarder>();
                forwarder.targetAgent = this;
                forwarder.showDebugLogs = showDebugLogs;
                forwardersAdded++;
            }
            else
            {
                existing.targetAgent = this;
            }
            
            processedObjects.Add(col.gameObject);
        }
        
        Debug.Log($"[{gameObject.name}] ✓ Collision detection: {forwardersAdded} forwarders on " +
                  $"{bodies.Length} ArticulationBodies + collider objects");
    }
    
    // ========================================================================
    // ON EPISODE BEGIN
    // ========================================================================
    
    public override void OnEpisodeBegin()
    {
        episodeStepCount = 0;
        episodeTotalReward = 0f;
        obstacleCollisionCount = 0;
        robotCollisionThisStep = false;
        obstacleCollisionThisStep = false;
        stuckCounter = 0;
        
        // Stop motion
        agvController?.SetMLCommands(0f, 0f);
        
        // Reset robot position
        if (rootArticulationBody != null && rootArticulationBody.isRoot)
        {
            rootArticulationBody.TeleportRoot(startPos, startRot);
            
            ArticulationBody[] bodies = rootArticulationBody.GetComponentsInChildren<ArticulationBody>();
            foreach (var body in bodies)
            {
                body.velocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }
        else
        {
            robotRoot.position = startPos;
            robotRoot.rotation = startRot;
        }
        
        lastPosition = robotRoot.position;
        
        // Reset shared coverage (only one agent should do this!)
        // We use a simple check: reset if this is the "first" agent by name
        if (coverageManager != null && gameObject.name.CompareTo(GetOtherAgentName()) < 0)
        {
            coverageManager.ResetCoverage();
        }
        
        if (showDebugLogs)
            Debug.Log($"[{gameObject.name}] Episode started");
    }
    
    private string GetOtherAgentName()
    {
        if (otherRobot != null)
        {
            var otherAgent = otherRobot.GetComponentInParent<TurtlebotCoverageAgent>();
            if (otherAgent != null)
                return otherAgent.gameObject.name;
        }
        return "zzz";  // Default that sorts last
    }
    
    // ========================================================================
    // COLLECT OBSERVATIONS (15 total)
    // ========================================================================
    
    public override void CollectObservations(VectorSensor sensor)
    {
        // ====================================================================
        // SELF STATE (4 observations)
        // ====================================================================
        
        // 1-2: Position (normalized)
        Vector3 pos = robotRoot.position;
        sensor.AddObservation(Mathf.Clamp(pos.x / boundaryLimit, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(pos.z / boundaryLimit, -1f, 1f));
        
        // 3-4: Forward direction
        Vector3 fwd = robotRoot.forward;
        sensor.AddObservation(fwd.x);
        sensor.AddObservation(fwd.z);
        
        // ====================================================================
        // OTHER ROBOT (3 observations)
        // ====================================================================
        
        if (otherRobot != null)
        {
            // Direction to other robot in LOCAL frame
            Vector3 toOther = otherRobot.position - robotRoot.position;
            Vector3 toOtherLocal = robotRoot.InverseTransformDirection(toOther);
            Vector3 toOtherNorm = toOtherLocal.normalized;
            
            // 5: Forward component (positive = other robot is ahead)
            sensor.AddObservation(toOtherNorm.z);
            
            // 6: Right component (positive = other robot is to the right)
            sensor.AddObservation(toOtherNorm.x);
            
            // 7: Distance (normalized, capped at 10m)
            float dist = toOther.magnitude;
            sensor.AddObservation(Mathf.Clamp01(dist / 10f));
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(1f);  // Far away
        }
        
        // ====================================================================
        // ENVIRONMENT (2 observations)
        // ====================================================================
        
        if (coverageManager != null)
        {
            // 8: Global coverage fraction
            sensor.AddObservation(coverageManager.GetCoverageFraction());
            
            // 9: Local unexplored density (how much unexplored nearby)
            sensor.AddObservation(coverageManager.GetLocalUnexploredDensity(robotRoot.position, 2f));
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }
        
        // ====================================================================
        // OBSTACLE SENSING - RAYCASTS (6 observations)
        // ====================================================================
        
        PerformRaycasts();
        
        for (int i = 0; i < numRays; i++)
        {
            // 10-15: Normalized distances (1 = far/no hit, 0 = very close)
            sensor.AddObservation(rayDistances[i]);
        }
        
        // Total: 4 + 3 + 2 + 6 = 15 observations
    }
    
    /// <summary>
    /// Cast rays in front of the robot to detect obstacles.
    /// </summary>
    private void PerformRaycasts()
    {
        Vector3 origin = robotRoot.position + Vector3.up * rayHeight;
        float minDetectedDistance = float.MaxValue;
        
        for (int i = 0; i < numRays; i++)
        {
            // Calculate angle for this ray
            float angle;
            if (numRays == 1)
                angle = 0f;
            else
                angle = -rayAngleSpan * 0.5f + rayAngleSpan * (i / (float)(numRays - 1));
            
            Quaternion rotation = Quaternion.AngleAxis(angle, Vector3.up);
            Vector3 direction = rotation * robotRoot.forward;
            
            // Cast ray
            if (Physics.Raycast(origin, direction, out RaycastHit hit, rayMaxDistance, obstacleLayerMask))
            {
                rayDistances[i] = hit.distance / rayMaxDistance;
                minDetectedDistance = Mathf.Min(minDetectedDistance, hit.distance);
                
                if (showRaycastDebug)
                    Debug.DrawLine(origin, hit.point, Color.red);
            }
            else
            {
                rayDistances[i] = 1f;
                
                if (showRaycastDebug)
                    Debug.DrawRay(origin, direction * rayMaxDistance, Color.green);
            }
        }
    }
    
    /// <summary>
    /// Get the minimum detected distance from raycasts.
    /// </summary>
    private float GetMinRayDistance()
    {
        float min = 1f;
        for (int i = 0; i < numRays; i++)
        {
            if (rayDistances[i] < min)
                min = rayDistances[i];
        }
        return min * rayMaxDistance;  // Convert back to meters
    }
    
    // ========================================================================
    // ON ACTION RECEIVED
    // ========================================================================
    
    public override void OnActionReceived(ActionBuffers actions)
    {
        episodeStepCount++;
        
        // ====================================================================
        // EXECUTE MOVEMENT
        // ====================================================================
        
        float forwardCmd = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float turnCmd = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        
        float linearVel = forwardCmd * maxLinearSpeed;
        float angularVel = turnCmd * maxAngularSpeed;
        
        if (agvController != null)
            agvController.SetMLCommands(linearVel, angularVel);
        
        // ====================================================================
        // CALCULATE REWARDS
        // ====================================================================
        
        float stepReward = 0f;
        
        // --- 1. Time penalty ---
        stepReward -= timeStepPenalty;
        
        // --- 2. Coverage reward (SHARED!) ---
        if (coverageManager != null)
        {
            bool newCell = coverageManager.RegisterVisit(robotRoot.position);
            if (newCell)
            {
                stepReward += newCellReward;
                
                if (showDebugLogs)
                    Debug.Log($"[{gameObject.name}] New cell! Coverage: {coverageManager.GetCoverageFraction() * 100:F1}%");
            }
        }
        
        // --- 3. Forward movement reward ---
        if (forwardCmd > 0.1f)
        {
            stepReward += forwardMovementReward * forwardCmd;
        }
        
        // --- 4. Proximity penalty (too close to obstacles) ---
        float minDist = GetMinRayDistance();
        if (minDist < dangerDistance)
        {
            float danger = 1f - (minDist / dangerDistance);  // 0 to 1
            stepReward -= proximityPenaltyScale * danger;
        }
        
        // --- 5. Other robot proximity penalty ---
        if (otherRobot != null)
        {
            float robotDist = Vector3.Distance(robotRoot.position, otherRobot.position);
            if (robotDist < 1.0f)  // Within 1 meter
            {
                float danger = 1f - robotDist;
                stepReward -= proximityPenaltyScale * danger * 2f;  // Extra penalty for robot
            }
        }
        
        // --- 6. Collision penalties ---
        if (obstacleCollisionThisStep)
        {
            stepReward -= obstacleCollisionPenalty;
            obstacleCollisionCount++;
            obstacleCollisionThisStep = false;
            
            // ALWAYS log obstacle collisions (important for debugging!)
            Debug.Log($"[{gameObject.name}] 🔴 OBSTACLE HIT! Count: {obstacleCollisionCount}/{maxObstacleCollisions}");
            
            if (obstacleCollisionCount >= maxObstacleCollisions)
            {
                AddReward(stepReward);
                Debug.LogWarning($"[{gameObject.name}] ❌ Episode ended: TOO MANY COLLISIONS ({obstacleCollisionCount}/{maxObstacleCollisions}). " +
                               $"Increase 'Max Obstacle Collisions' in Inspector for more lenient testing!");
                EndEpisode();
                return;
            }
        }
        
        if (robotCollisionThisStep)
        {
            stepReward -= robotCollisionPenalty;
            robotCollisionThisStep = false;
            
            // ALWAYS log robot-robot collisions
            Debug.Log($"[{gameObject.name}] 🤖💥🤖 ROBOT COLLISION!");
            
            if (endOnRobotCollision)
            {
                AddReward(stepReward);
                Debug.LogWarning($"[{gameObject.name}] ❌ Episode ended: ROBOT COLLISION. " +
                               $"Uncheck 'End On Robot Collision' in Inspector for more lenient testing!");
                EndEpisode();
                return;
            }
        }
        
        // --- 7. Stuck detection ---
        // Only check if robot is TRYING to move (command > threshold)
        float moved = Vector3.Distance(robotRoot.position, lastPosition);
        lastPosition = robotRoot.position;
            
        // Only count trying to move if there is forward/backward command.
        // Pure rotation is allowed and should NOT trigger stuck logic.
        bool isTryingToMove = Mathf.Abs(forwardCmd) > 0.1f;

        
        if (isTryingToMove && moved < stuckThreshold)
        {
            // Robot is trying to move but can't - might be stuck
            stuckCounter++;
            if (stuckCounter >= stuckStepsLimit)
            {
                stepReward -= stuckPenalty;
                Debug.LogWarning($"[{gameObject.name}] Episode ended: STUCK for {stuckStepsLimit} steps. " +
                               $"Robot can't move! Maybe wedged against obstacle. " +
                               $"Increase 'Stuck Steps Limit' in Inspector or check robot position.");
                AddReward(stepReward);
                EndEpisode();
                return;
            }
        }
        else
        {
            // Reset counter if moving OR if not trying to move
            stuckCounter = 0;
        }
        
        // --- 8. Boundary check ---
        Vector3 pos = robotRoot.position;
        if (Mathf.Abs(pos.x) > boundaryLimit || Mathf.Abs(pos.z) > boundaryLimit)
        {
            stepReward -= outOfBoundsPenalty;
            // ALWAYS log boundary violations - this is a common source of confusion
            Debug.LogWarning($"[{gameObject.name}] Episode ended: OUT OF BOUNDS! " +
                           $"Position=({pos.x:F1}, {pos.z:F1}), Limit=±{boundaryLimit}. " +
                           $"Increase 'Boundary Limit' in Inspector!");
            AddReward(stepReward);
            EndEpisode();
            return;
        }
        
        // Apply total reward for this step
        AddReward(stepReward);
        episodeTotalReward += stepReward;
        
        // Debug logging
        if (logRewardBreakdown && episodeStepCount % 100 == 0)
        {
            float coverage = coverageManager != null ? coverageManager.GetCoverageFraction() : 0f;
            Debug.Log($"[{gameObject.name}] Step {episodeStepCount}: " +
                      $"Reward={episodeTotalReward:F2}, Coverage={coverage * 100:F1}%, " +
                      $"MinDist={minDist:F2}m");
        }
    }
    
    // ========================================================================
    // HEURISTIC (Keyboard Control)
    // ========================================================================
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        cont[0] = Input.GetAxis("Vertical");
        cont[1] = Input.GetAxis("Horizontal");
    }
    
    // ========================================================================
    // COLLISION DETECTION
    // ========================================================================
    
    // These are kept for cases where the agent script IS on the same object as colliders
    private void OnCollisionEnter(Collision collision)
    {
        HandleCollision(collision.gameObject);
    }
    
    private void OnCollisionStay(Collision collision)
    {
        HandleCollision(collision.gameObject);
    }
    
    /// <summary>
    /// PUBLIC method called by CollisionForwarder scripts on child objects.
    /// This is needed because Unity collision callbacks only fire on GameObjects
    /// that have the Collider component, not parent GameObjects.
    /// </summary>
    public void ForwardCollision(GameObject other)
    {
        HandleCollision(other);
    }
    
    private void HandleCollision(GameObject other)
    {   
        // Quick check for floor/ground - skip logging and processing entirely
        string nameLower = other.name.ToLower();
        if (nameLower.Contains("floor") || nameLower.Contains("ground") || 
            nameLower.Contains("plane") || nameLower.Contains("ceiling"))
        {
            return;  // Ignore floor collisions completely - we're always touching it!
        }
        
        // Log non-floor collisions for debugging
        if (showDebugLogs)
        {
            Debug.Log($"[{gameObject.name}] 💥 COLLISION with '{other.name}' tag='{other.tag}' layer='{LayerMask.LayerToName(other.layer)}'");
        }

        // Check if it's the other robot
        if (otherRobot != null && 
            (other.transform == otherRobot || 
             other.transform.IsChildOf(otherRobot) || 
             otherRobot.IsChildOf(other.transform)))
        {
            if (showDebugLogs)
                Debug.Log($"[{gameObject.name}] Robot collision detected with: {other.name}");
            robotCollisionThisStep = true;
            return;
        }
        
        // Check if it's self
        if (other.transform.IsChildOf(robotRoot) || robotRoot.IsChildOf(other.transform))
            return;
        
        // Note: Floor/ground check already done at top of function (early return)
        
        // Check if it's tagged as "Robot" (ignore, handled above)
        if (other.CompareTag("Robot"))
            return;
        
        // TAG-BASED COLLISION DETECTION
        if (useTagBasedCollision)
        {
            // Check if object OR ANY PARENT has an obstacle tag
            bool isObstacle = CheckTagInHierarchy(other.transform);
            
            if (isObstacle)
            {
                if (showDebugLogs)
                    Debug.Log($"[{gameObject.name}] Obstacle collision: {other.name} (tag found in hierarchy)");
                obstacleCollisionThisStep = true;
            }
            else if (showDebugLogs)
            {
                Debug.Log($"[{gameObject.name}] Collision IGNORED: {other.name} has no obstacle tag (checked parents too)");
            }
        }
        else
        {
            // Old behavior: everything not floor/self is an obstacle
            if (showDebugLogs)
                Debug.Log($"[{gameObject.name}] Obstacle collision: {other.name} (non-tag mode)");
            obstacleCollisionThisStep = true;
        }
    }
    
    /// <summary>
    /// Checks if this transform or ANY of its parents have one of our obstacle tags.
    /// This solves the problem where parent has the tag but children have the colliders.
    /// </summary>
    private bool CheckTagInHierarchy(Transform obj)
    {
        Transform current = obj;
        
        // Walk up the hierarchy checking tags
        while (current != null)
        {
            foreach (string tag in obstacleTags)
            {
                if (current.CompareTag(tag))
                {
                    return true;
                }
            }
            current = current.parent;
        }
        
        return false;
    }
    
    // ========================================================================
    // DEBUG VISUALIZATION
    // ========================================================================
    
    private void OnDrawGizmosSelected()
    {
        if (robotRoot == null) return;
        
        // Boundary
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(boundaryLimit * 2, 0.1f, boundaryLimit * 2));
        
        // Danger zone
        Gizmos.color = new Color(1f, 0.5f, 0f, 0.3f);
        Gizmos.DrawWireSphere(robotRoot.position, dangerDistance);
        
        // Line to other robot
        if (otherRobot != null)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(robotRoot.position, otherRobot.position);
        }
    }
}

