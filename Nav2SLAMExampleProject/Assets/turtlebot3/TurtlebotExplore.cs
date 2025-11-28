// ============================================================================
// TurtlebotExplore.cs - ML-Agents Reinforcement Learning Agent for Turtlebot
// ============================================================================
//
// This script teaches a Turtlebot robot to navigate to a target using
// Reinforcement Learning (RL). It uses Unity ML-Agents and the PPO algorithm.
//
// ============================================================================
// HOW REINFORCEMENT LEARNING WORKS (Simple Explanation):
// ============================================================================
//
// Imagine teaching a dog to fetch a ball:
// 1. The dog SEES the world (observations)
// 2. The dog DOES something (actions) 
// 3. You give the dog a TREAT or SCOLD it (rewards)
// 4. The dog LEARNS what actions lead to treats
//
// In our case:
// - The robot SEES: where the target is, how far away, which direction it's facing
// - The robot DOES: move forward/backward, turn left/right
// - We give REWARDS: +2 for reaching target, -1 for going out of bounds
// - The robot LEARNS: "if target is to my right, I should turn right"
//
// ============================================================================
// THE TRAINING LOOP (What happens during training):
// ============================================================================
//
// 1. EPISODE STARTS (OnEpisodeBegin):
//    - Robot teleports back to starting position
//    - Target moves to a new random location
//    - This is like resetting a game level
//
// 2. EVERY STEP (repeated many times per second):
//    a) OBSERVE (CollectObservations):
//       - Robot measures: "Where is target relative to me?"
//       - These 7 numbers go into a neural network
//
//    b) DECIDE (Neural Network - handled by ML-Agents):
//       - The neural network outputs 2 numbers: forward speed, turn speed
//       - Early in training: random guesses
//       - Later: smart decisions based on learned patterns
//
//    c) ACT (OnActionReceived):
//       - Robot executes the movement commands
//       - Wheels spin via AGVController
//
//    d) REWARD (OnActionReceived):
//       - Did robot get closer to target? +reward
//       - Did robot reach target? +2.0 and episode ends
//       - Did robot go out of bounds? -1.0 and episode ends
//
// 3. EPISODE ENDS when:
//    - Robot reaches target (SUCCESS!)
//    - Robot goes out of bounds (FAILURE)
//    - Too many steps pass (TIMEOUT)
//
// 4. LEARNING (handled by ML-Agents Python):
//    - PPO algorithm analyzes all the episodes
//    - Updates neural network weights
//    - Actions that led to high rewards become more likely
//    - Actions that led to low rewards become less likely
//
// 5. REPEAT for millions of steps until robot masters the task
//
// ============================================================================

using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using RosSharp.Control;

/// <summary>
/// ML-Agents Agent that learns to navigate a Turtlebot to reach a target.
/// 
/// Inherits from Agent class which provides:
/// - Initialize(): Called once when training starts
/// - OnEpisodeBegin(): Called at start of each episode
/// - CollectObservations(): Called to gather what the agent "sees"
/// - OnActionReceived(): Called when neural network outputs actions
/// - Heuristic(): Called for manual keyboard control (testing)
/// </summary>
public class TurtlebotExplore : Agent
{
    // ========================================================================
    // INSPECTOR SETTINGS - Configure these in Unity Editor
    // ========================================================================
    
    [Header("Robot References")]
    [Tooltip("The transform representing the robot's base (usually base_footprint)")]
    public Transform robotRoot;
    
    [Tooltip("Reference to AGVController that controls the wheels")]
    public AGVController agvController;
    
    [Header("Movement Limits")]
    [Tooltip("Maximum forward/backward speed in meters per second")]
    public float maxLinearSpeed = 0.5f;
    
    [Tooltip("Maximum turning speed in radians per second")]
    public float maxAngularSpeed = 1.0f;

    [Header("Target Settings")]
    [Tooltip("The target object the robot should reach")]
    public Transform target;
    
    [Tooltip("How close the robot must get to 'reach' the target (meters)")]
    public float targetReachDistance = 0.5f;
    
    [Tooltip("How far the target can spawn from its original position")]
    public float targetRandomizeRadius = 2.5f;

    [Header("Environment Bounds")]
    [Tooltip("If robot goes beyond this distance from origin, episode ends")]
    public float boundaryLimit = 5.5f;

    [Header("Reward Tuning - The 'treats' and 'scoldings'")]
    [Tooltip("Small negative reward every step (encourages finishing quickly)")]
    public float timeStepPenalty = -0.0005f;
    
    [Tooltip("Reward multiplier for getting closer to target")]
    public float progressRewardScale = 0.1f;
    
    [Tooltip("Negative reward when touching obstacles")]
    public float collisionPenalty = -0.02f;
    
    [Tooltip("Big reward for reaching the target")]
    public float targetReachedReward = 2.0f;
    
    [Tooltip("Big penalty for going out of bounds")]
    public float outOfBoundsPenalty = -1.0f;
    
    [Tooltip("If true, episode ends immediately on collision")]
    public bool endEpisodeOnCollision = false;

    [Header("Debug")]
    [Tooltip("Enable to see debug messages in Console")]
    public bool showDebugLogs = false;

    // ========================================================================
    // PRIVATE STATE - Internal variables the agent tracks
    // ========================================================================
    
    private Vector3 startPos;                    // Where robot spawns each episode
    private Quaternion startRot;                 // Robot's starting rotation
    private Vector3 targetStartPos;              // Original target position
    private float previousDistanceToTarget;      // Distance last frame (for progress reward)
    private ArticulationBody rootArticulationBody; // Physics body for proper reset
    private bool isColliding = false;            // Is robot touching an obstacle?
    private int episodeStepCount = 0;            // How many steps in current episode

    // ========================================================================
    // INITIALIZE - Called once when the game starts
    // ========================================================================
    /// <summary>
    /// Called once at the very beginning. Sets up references and stores
    /// initial positions for episode resets.
    /// </summary>
    public override void Initialize()
    {
        // Auto-find robot root if not assigned in Inspector
        if (robotRoot == null)
            robotRoot = transform;

        // Auto-find AGVController if not assigned
        if (agvController == null)
            agvController = GetComponent<AGVController>();

        // Find the ArticulationBody for proper physics-based reset
        // ArticulationBody is Unity's physics component for robot joints
        rootArticulationBody = robotRoot.GetComponentInParent<ArticulationBody>();
        if (rootArticulationBody == null)
            rootArticulationBody = robotRoot.GetComponent<ArticulationBody>();

        // Store initial positions - we'll teleport back here each episode
        startPos = robotRoot.position;
        startRot = robotRoot.rotation;

        if (target != null)
            targetStartPos = target.position;

        if (showDebugLogs)
            Debug.Log($"[TurtlebotExplore] Initialized. ArticulationBody found: {rootArticulationBody != null}");
    }

    // ========================================================================
    // ON EPISODE BEGIN - Called at the start of each training episode
    // ========================================================================
    /// <summary>
    /// Called when a new episode starts (after reaching target, going out of
    /// bounds, or timeout). Resets the robot and randomizes the target.
    /// 
    /// Think of this as "resetting the game level" after each attempt.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        // Reset episode counters
        episodeStepCount = 0;
        isColliding = false;

        // Stop any existing motion
        agvController?.SetMLCommands(0f, 0f);

        // ====================================================================
        // RESET ROBOT POSITION
        // ====================================================================
        // For URDF robots with ArticulationBody, we must use TeleportRoot()
        // because directly setting transform.position doesn't work properly
        // with physics joints.
        
        if (rootArticulationBody != null && rootArticulationBody.isRoot)
        {
            // TeleportRoot teleports the entire robot hierarchy at once
            rootArticulationBody.TeleportRoot(startPos, startRot);
            
            // Also reset all velocities to prevent momentum carrying over
            ArticulationBody[] allBodies = rootArticulationBody.GetComponentsInChildren<ArticulationBody>();
            foreach (var body in allBodies)
            {
                body.velocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }
        else
        {
            // Fallback for simple Rigidbody setups
            robotRoot.position = startPos;
            robotRoot.rotation = startRot;
            
            Rigidbody rb = robotRoot.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
        }

        // ====================================================================
        // RANDOMIZE TARGET POSITION
        // ====================================================================
        // Each episode, the target appears at a different location.
        // This teaches the robot to generalize - not just memorize one path.
        
        if (target != null)
        {
            Vector3 newTargetPos;
            int attempts = 0;
            const int maxAttempts = 10;
            
            // Try to find a valid position (within bounds, not too close to robot)
            do
            {
                newTargetPos = new Vector3(
                    targetStartPos.x + Random.Range(-targetRandomizeRadius, targetRandomizeRadius),
                    targetStartPos.y,  // Keep same height
                    targetStartPos.z + Random.Range(-targetRandomizeRadius, targetRandomizeRadius)
                );
                attempts++;
            }
            while (attempts < maxAttempts && 
                   (Mathf.Abs(newTargetPos.x) > boundaryLimit - 0.5f || 
                    Mathf.Abs(newTargetPos.z) > boundaryLimit - 0.5f ||
                    Vector3.Distance(newTargetPos, startPos) < 1.0f));
            
            target.position = newTargetPos;
        }

        // Store initial distance for progress reward calculation
        previousDistanceToTarget = GetDistanceToTarget();

        if (showDebugLogs)
            Debug.Log($"[TurtlebotExplore] Episode started. Initial distance: {previousDistanceToTarget:F2}");
    }

    // ========================================================================
    // COLLECT OBSERVATIONS - What the robot "sees"
    // ========================================================================
    /// <summary>
    /// Called every decision step. Returns what the agent can "see".
    /// 
    /// These observations go into the neural network as inputs.
    /// The neural network learns patterns like:
    /// "When observation[3] (angle to target) is positive, I should output
    /// positive turn command to turn right toward the target."
    /// 
    /// IMPORTANT: All observations are normalized to roughly [-1, 1] range.
    /// This helps the neural network learn faster.
    /// 
    /// Total: 7 observations (must match Behavior Parameters in Inspector!)
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        // ====================================================================
        // ROBOT-RELATIVE OBSERVATIONS (Observations 1-4)
        // ====================================================================
        // We express target direction in the ROBOT's frame of reference.
        // This is crucial! Instead of "target is at world position (5,3)",
        // we say "target is 2 meters ahead and 1 meter to my right".
        // 
        // Why? Because the robot's actions (forward, turn) are also relative
        // to itself. This makes the learning problem much simpler.
        
        if (target != null)
        {
            // Step 1: Get vector from robot to target in WORLD coordinates
            Vector3 toTargetWorld = target.position - robotRoot.position;
            float distance = toTargetWorld.magnitude;
            
            // Step 2: Transform to ROBOT's LOCAL coordinates
            // InverseTransformDirection converts world direction to local direction
            // After this: toTargetLocal.z = forward/back, toTargetLocal.x = left/right
            Vector3 toTargetLocal = robotRoot.InverseTransformDirection(toTargetWorld);
            
            // Observation 1: Forward component (is target ahead or behind?)
            // +1 = target is directly ahead, -1 = target is directly behind
            Vector3 localDirNorm = toTargetLocal.normalized;
            sensor.AddObservation(localDirNorm.z);
            
            // Observation 2: Right component (is target left or right?)
            // +1 = target is to the right, -1 = target is to the left
            sensor.AddObservation(localDirNorm.x);
            
            // Observation 3: Distance to target (normalized)
            // Divided by 10 to keep in [0, 1] range
            sensor.AddObservation(Mathf.Clamp01(distance / 10f));
            
            // Observation 4: Angle to target (in degrees, normalized)
            // This directly tells the agent "turn this much to face target"
            // Positive = target is to the right, negative = to the left
            float angleToTarget = Mathf.Atan2(toTargetLocal.x, toTargetLocal.z) * Mathf.Rad2Deg;
            sensor.AddObservation(angleToTarget / 180f);  // Normalize to [-1, 1]
        }
        else
        {
            // No target - send zeros
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }
        
        // ====================================================================
        // WORLD POSITION (Observations 5-6)
        // ====================================================================
        // The robot needs to know where it is to avoid going out of bounds.
        // Normalized by boundaryLimit so values stay in [-1, 1] range.
        
        Vector3 pos = robotRoot.position;
        
        // Observation 5: Robot's X position relative to boundary
        sensor.AddObservation(Mathf.Clamp(pos.x / boundaryLimit, -1f, 1f));
        
        // Observation 6: Robot's Z position relative to boundary  
        sensor.AddObservation(Mathf.Clamp(pos.z / boundaryLimit, -1f, 1f));
        
        // ====================================================================
        // ANGULAR VELOCITY (Observation 7)
        // ====================================================================
        // How fast the robot is currently spinning.
        // This helps the agent control smooth movements and avoid oscillation.
        
        ArticulationBody ab = robotRoot.GetComponentInParent<ArticulationBody>();
        if (ab != null)
        {
            float angularVelY = ab.angularVelocity.y;
            sensor.AddObservation(Mathf.Clamp(angularVelY / 2f, -1f, 1f));
        }
        else
        {
            sensor.AddObservation(0f);
        }
        
        // Total: 7 observations
    }

    // ========================================================================
    // ON ACTION RECEIVED - Execute actions and give rewards
    // ========================================================================
    /// <summary>
    /// Called when the neural network outputs actions.
    /// 
    /// This method:
    /// 1. Executes the movement commands
    /// 2. Calculates and gives rewards
    /// 3. Checks if episode should end
    /// 
    /// The rewards are the "teaching signal" - they tell the neural network
    /// whether its actions were good or bad.
    /// </summary>
    public override void OnActionReceived(ActionBuffers actions)
    {
        episodeStepCount++;

        // ====================================================================
        // STEP 1: EXECUTE ACTIONS
        // ====================================================================
        // The neural network outputs 2 continuous values in range [-1, 1]:
        // - actions.ContinuousActions[0]: Forward/backward command
        // - actions.ContinuousActions[1]: Turn left/right command
        
        float forwardCmd = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float turnCmd = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        // Convert to actual velocity values
        // forwardCmd of 1.0 → maxLinearSpeed m/s forward
        // forwardCmd of -1.0 → maxLinearSpeed m/s backward
        float linearVel = forwardCmd * maxLinearSpeed;
        float angularVel = turnCmd * maxAngularSpeed;

        // Send to AGVController which converts to wheel speeds
        if (agvController != null)
            agvController.SetMLCommands(linearVel, angularVel);

        // ====================================================================
        // STEP 2: CALCULATE REWARDS
        // ====================================================================
        // Rewards shape what the agent learns. They're like grades on a test.
        
        // --------------------------------------------------------------------
        // Reward 1: Time Penalty (small negative each step)
        // --------------------------------------------------------------------
        // Purpose: Encourage the robot to reach the target quickly.
        // Without this, the robot might wander around forever.
        AddReward(timeStepPenalty);  // e.g., -0.0005 per step

        // --------------------------------------------------------------------
        // Reward 2: Progress Reward (positive for getting closer)
        // --------------------------------------------------------------------
        // This is the KEY reward! It teaches the robot that moving toward
        // the target is good.
        // 
        // distanceDelta = previousDistance - currentDistance
        // If robot got closer: distanceDelta > 0 → positive reward
        // If robot got further: distanceDelta < 0 → negative reward
        
        float currentDistance = GetDistanceToTarget();
        float distanceDelta = previousDistanceToTarget - currentDistance;
        float progressReward = distanceDelta * progressRewardScale;
        AddReward(progressReward);
        previousDistanceToTarget = currentDistance;

        // --------------------------------------------------------------------
        // Reward 3: Spin Penalty (small negative for turning)
        // --------------------------------------------------------------------
        // Purpose: Discourage spinning in circles. Encourages smooth movement.
        float spinPenalty = Mathf.Abs(turnCmd) * 0.0001f;
        AddReward(-spinPenalty);

        // --------------------------------------------------------------------
        // Reward 4: Facing Bonus (reward for facing target while moving)
        // --------------------------------------------------------------------
        // Purpose: Encourage the robot to point toward target before driving.
        if (target != null && forwardCmd > 0.1f)
        {
            Vector3 toTarget = (target.position - robotRoot.position).normalized;
            // Dot product: 1 if perfectly aligned, -1 if facing opposite
            float facingBonus = Vector3.Dot(robotRoot.forward, toTarget);
            if (facingBonus > 0.7f)  // If roughly aligned (within ~45 degrees)
            {
                AddReward(0.001f * facingBonus);
            }
        }

        // --------------------------------------------------------------------
        // Reward 5: Collision Penalty
        // --------------------------------------------------------------------
        // Purpose: Teach robot to avoid obstacles.
        if (isColliding)
        {
            AddReward(collisionPenalty);  // e.g., -0.02 per step while colliding
            
            if (endEpisodeOnCollision)
            {
                if (showDebugLogs)
                    Debug.Log($"[TurtlebotExplore] Episode ended: Collision");
                EndEpisode();
                return;
            }
        }

        // ====================================================================
        // STEP 3: CHECK EPISODE END CONDITIONS
        // ====================================================================
        
        // --------------------------------------------------------------------
        // Success: Robot reached the target!
        // --------------------------------------------------------------------
        if (currentDistance < targetReachDistance)
        {
            // Give big reward! Also bonus for reaching quickly.
            float speedBonus = Mathf.Clamp01(1f - (episodeStepCount / 500f)) * 0.5f;
            AddReward(targetReachedReward + speedBonus);
            
            if (showDebugLogs)
                Debug.Log($"[TurtlebotExplore] TARGET REACHED! Steps: {episodeStepCount}, SpeedBonus: {speedBonus:F3}");
            
            // End this episode, start a new one
            EndEpisode();
            return;
        }

        // --------------------------------------------------------------------
        // Failure: Robot went out of bounds
        // --------------------------------------------------------------------
        Vector3 pos = robotRoot.position;
        if (Mathf.Abs(pos.x) > boundaryLimit || Mathf.Abs(pos.z) > boundaryLimit)
        {
            AddReward(outOfBoundsPenalty);  // e.g., -1.0
            
            if (showDebugLogs)
                Debug.Log($"[TurtlebotExplore] Episode ended: Out of bounds at ({pos.x:F2}, {pos.z:F2})");
            
            EndEpisode();
            return;
        }

        // Optional debug logging every 50 steps
        if (showDebugLogs && episodeStepCount % 50 == 0)
        {
            Debug.Log($"[TurtlebotExplore] Step {episodeStepCount}: dist={currentDistance:F2}, " +
                      $"progress={progressReward:F4}, fwd={forwardCmd:F2}, turn={turnCmd:F2}");
        }
    }

    // ========================================================================
    // HEURISTIC - Manual keyboard control for testing
    // ========================================================================
    /// <summary>
    /// Called when Behavior Type is set to "Heuristic Only" in Inspector.
    /// Allows you to control the robot with keyboard for testing.
    /// 
    /// This is useful for:
    /// - Testing if the robot moves correctly
    /// - Understanding what actions do
    /// - Comparing human performance to trained agent
    /// </summary>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        
        // W/S keys → forward/backward (action[0])
        cont[0] = Input.GetAxis("Vertical");
        
        // A/D keys → turn left/right (action[1])
        cont[1] = Input.GetAxis("Horizontal");
    }

    // ========================================================================
    // COLLISION DETECTION - Detect when robot hits obstacles
    // ========================================================================
    // These Unity callbacks are triggered automatically when physics collisions occur.
    
    /// <summary>Called when robot first touches an object</summary>
    private void OnCollisionEnter(Collision collision)
    {
        if (IsObstacle(collision.gameObject))
        {
            isColliding = true;
            if (showDebugLogs)
                Debug.Log($"[TurtlebotExplore] Collision with: {collision.gameObject.name}");
        }
    }

    /// <summary>Called every frame while robot is touching an object</summary>
    private void OnCollisionStay(Collision collision)
    {
        if (IsObstacle(collision.gameObject))
        {
            isColliding = true;
        }
    }

    /// <summary>Called when robot stops touching an object</summary>
    private void OnCollisionExit(Collision collision)
    {
        if (IsObstacle(collision.gameObject))
        {
            isColliding = false;
        }
    }

    /// <summary>
    /// Determines if a collided object counts as an "obstacle" for penalty purposes.
    /// We ignore floors, robot's own parts, and the target.
    /// </summary>
    private bool IsObstacle(GameObject obj)
    {
        string nameLower = obj.name.ToLower();
        
        // Ignore floor/ground (we're always touching it!)
        if (nameLower.Contains("floor") || nameLower.Contains("ground") || 
            nameLower.Contains("plane") || nameLower.Contains("ceiling"))
            return false;

        // Ignore self-collision (robot's own parts)
        if (obj.transform.IsChildOf(robotRoot) || robotRoot.IsChildOf(obj.transform))
            return false;

        // Ignore the target object
        if (target != null && obj.transform == target)
            return false;

        // Everything else is an obstacle (boxes, shelves, walls)
        return true;
    }

    // ========================================================================
    // HELPER METHODS
    // ========================================================================
    
    /// <summary>Calculate distance from robot to target</summary>
    private float GetDistanceToTarget()
    {
        if (target == null)
            return 0f;
        return Vector3.Distance(robotRoot.position, target.position);
    }

    /// <summary>
    /// Draw debug visuals in Scene view when object is selected.
    /// Shows boundary box and line to target.
    /// </summary>
    private void OnDrawGizmosSelected()
    {
        if (robotRoot == null) return;

        // Draw boundary box
        Gizmos.color = Color.yellow;
        Vector3 center = new Vector3(0, robotRoot.position.y, 0);
        Gizmos.DrawWireCube(center, new Vector3(boundaryLimit * 2, 0.1f, boundaryLimit * 2));

        // Draw line from robot to target
        if (target != null)
        {
            Gizmos.color = isColliding ? Color.red : Color.green;
            Gizmos.DrawLine(robotRoot.position, target.position);
            
            // Draw target reach distance
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(target.position, targetReachDistance);
        }
    }
}
