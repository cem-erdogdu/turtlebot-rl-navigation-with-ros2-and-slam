// ============================================================================
// MapCoverageManager.cs - Shared Coverage Grid for Multi-Robot Exploration
// ============================================================================
//
// PURPOSE:
// This script creates a SHARED coverage grid that BOTH robots contribute to.
// When either robot visits a new cell, BOTH robots get rewarded (cooperation!).
//
// WHY SHARED REWARD?
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ If Robot1 explores area A and Robot2 explores area B:                  │
// │ - Total coverage increases quickly                                      │
// │ - BOTH robots get credit for the team's progress                       │
// │                                                                         │
// │ If Robot1 and Robot2 follow each other:                                │
// │ - Coverage increases slowly (same cells)                               │
// │ - Time penalty accumulates                                              │
// │ - They learn to SPREAD OUT for better team reward                      │
// └─────────────────────────────────────────────────────────────────────────┘
//
// ============================================================================

using UnityEngine;
using System.Collections.Generic;

public class MapCoverageManager : MonoBehaviour
{
    // ========================================================================
    // SINGLETON PATTERN - Easy access from any agent
    // ========================================================================
    public static MapCoverageManager Instance { get; private set; }
    
    // ========================================================================
    // CONFIGURATION
    // ========================================================================
    
    [Header("Coverage Area (World Coordinates)")]
    [Tooltip("Bottom-left corner of tracking area (X, Z)")]
    public Vector2 areaMin = new Vector2(-6f, -6f);
    
    [Tooltip("Top-right corner of tracking area (X, Z)")]
    public Vector2 areaMax = new Vector2(6f, 6f);
    
    [Tooltip("Size of each grid cell in meters")]
    public float cellSize = 0.5f;
    
    [Header("Robots (Auto-populated if empty)")]
    [Tooltip("All robots that contribute to coverage")]
    public List<Transform> robots = new List<Transform>();
    
    [Header("Coverage Tracking")]
    [Tooltip("Track coverage changes per step for reward calculation")]
    public bool trackCoverageDeltas = true;
    
    [Header("Obstacle Detection")]
    [Tooltip("Scan for obstacles at startup and mark blocked cells")]
    public bool scanForObstacles = true;
    
    [Tooltip("Tags that count as obstacles (will block cells). Only include tags that EXIST in your project!")]
    public string[] obstacleTags = { "Obstacle", "Shelf", "Box" };  // Removed "Wall" - add it back if you create the tag
    
    [Tooltip("Also find obstacles by name patterns (partial match)")]
    public string[] obstacleNamePatterns = { "Shelving", "Rack", "box", "Box" };
    
    [Tooltip("Name patterns to EXCLUDE from obstacles (robots, floors, etc.)")]
    public string[] excludeNamePatterns = { "robot", "Robot", "wheel", "caster", "base_", "floor", "Floor", 
                                            "ground", "Ground", "ceiling", "Ceiling", "plane", "Plane" };
    
    [Tooltip("Update blocked cells every frame (for moving obstacles)")]
    public bool realtimeObstacleTracking = false;
    
    [Tooltip("How often to update obstacle tracking (seconds)")]
    public float obstacleUpdateInterval = 0.5f;
    
    [Header("Manual Controls")]
    [Tooltip("Set to true and it will trigger a rescan on next frame")]
    public bool triggerRescan = false;
    
    private float lastObstacleUpdateTime = 0f;
    
    [Header("Debug Visualization")]
    public bool showDebugGrid = true;
    public bool showOnlyVisitedCells = false;
    public Color unvisitedColor = new Color(1f, 0f, 0f, 0.2f);
    public Color visitedColor = new Color(0f, 1f, 0f, 0.4f);
    public Color blockedColor = new Color(0.3f, 0.3f, 0.3f, 0.3f);
    
    // ========================================================================
    // INTERNAL STATE
    // ========================================================================
    
    private bool[,] visited;
    private bool[,] blocked;  // Cells that are occupied by obstacles
    private int cellsX, cellsZ;
    private int visitedCount;
    private int totalCount;      // Total cells in grid
    private int reachableCount;  // Cells that are NOT blocked (can be visited)
    private int blockedCount;    // Cells that are blocked by obstacles
    
    // For tracking coverage changes between steps
    private float lastCoverageFraction;
    private int newCellsThisStep;
    
    // ========================================================================
    // UNITY LIFECYCLE
    // ========================================================================
    
    void Awake()
    {
        // Singleton setup
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }
        Instance = this;
        
        // Initialize grid structure but DON'T scan yet
        // (obstacles may not be spawned yet during Awake)
        InitializeGridStructure();
    }
    
    void Start()
    {
        // Auto-find robots if none assigned
        // Look for objects tagged as "Robot" or with specific names
        if (robots.Count == 0)
        {
            // Try to find by tag first
            GameObject[] robotObjects = GameObject.FindGameObjectsWithTag("Robot");
            foreach (var robotObj in robotObjects)
            {
                Transform baseFootprint = robotObj.transform.Find("base_footprint");
                if (baseFootprint != null)
                    robots.Add(baseFootprint);
                else
                    robots.Add(robotObj.transform);
            }
            
            // If no tagged robots, try finding by name pattern
            if (robots.Count == 0)
            {
                GameObject robot1 = GameObject.Find("robot1");
                GameObject robot2 = GameObject.Find("robot2");
                
                if (robot1 != null)
                {
                    Transform bf1 = robot1.transform.Find("base_footprint");
                    robots.Add(bf1 != null ? bf1 : robot1.transform);
                }
                if (robot2 != null)
                {
                    Transform bf2 = robot2.transform.Find("base_footprint");
                    robots.Add(bf2 != null ? bf2 : robot2.transform);
                }
            }
            
            Debug.Log($"[MapCoverageManager] Auto-found {robots.Count} robots");
        }
        
        // NOW scan for obstacles - after all Start() methods have run
        // Use Invoke to delay slightly, ensuring all spawners have finished
        Invoke(nameof(DelayedObstacleScan), 0.1f);
    }
    
    private void DelayedObstacleScan()
    {
        if (scanForObstacles)
        {
            ScanForObstacles();
            Debug.Log($"[MapCoverageManager] Delayed obstacle scan triggered");
        }
    }
    
    void Update()
    {
        // Check for manual rescan trigger
        if (triggerRescan)
        {
            triggerRescan = false;
            RescanObstacles();
        }
        
        // Real-time obstacle tracking (for moving obstacles like falling boxes)
        if (realtimeObstacleTracking && Time.time - lastObstacleUpdateTime > obstacleUpdateInterval)
        {
            lastObstacleUpdateTime = Time.time;
            UpdateObstacleTracking();
        }
    }
    
    /// <summary>
    /// Lightweight update for tracking moving obstacles.
    /// Only updates cells, doesn't log to console.
    /// Reuses the same IsObstacle and ShouldExclude logic.
    /// </summary>
    private void UpdateObstacleTracking()
    {
        // Clear blocked status
        for (int x = 0; x < cellsX; x++)
            for (int z = 0; z < cellsZ; z++)
                blocked[x, z] = false;
        
        blockedCount = 0;
        
        // Find all colliders with obstacle tags/names
        Collider[] allColliders = FindObjectsOfType<Collider>();
        
        foreach (Collider col in allColliders)
        {
            if (col.isTrigger) continue;
            
            // Skip robots, floors, ceilings
            if (ShouldExcludeFromObstacles(col)) continue;
            
            // Check if it's an obstacle
            if (!IsObstacle(col)) continue;
            
            // Mark cells as blocked
            Bounds bounds = col.bounds;
            int minCellX = Mathf.Max(0, Mathf.FloorToInt((bounds.min.x - areaMin.x) / cellSize));
            int maxCellX = Mathf.Min(cellsX - 1, Mathf.CeilToInt((bounds.max.x - areaMin.x) / cellSize));
            int minCellZ = Mathf.Max(0, Mathf.FloorToInt((bounds.min.z - areaMin.y) / cellSize));
            int maxCellZ = Mathf.Min(cellsZ - 1, Mathf.CeilToInt((bounds.max.z - areaMin.y) / cellSize));
            
            for (int x = minCellX; x <= maxCellX; x++)
            {
                for (int z = minCellZ; z <= maxCellZ; z++)
                {
                    if (!blocked[x, z])
                    {
                        blocked[x, z] = true;
                        blockedCount++;
                    }
                }
            }
        }
        
        reachableCount = totalCount - blockedCount;
    }
    
    void FixedUpdate()
    {
        // Reset new cells counter each physics step
        newCellsThisStep = 0;
    }
    
    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    
    /// <summary>
    /// Initialize grid arrays - called during Awake (before obstacles spawn)
    /// </summary>
    private void InitializeGridStructure()
    {
        cellsX = Mathf.CeilToInt((areaMax.x - areaMin.x) / cellSize);
        cellsZ = Mathf.CeilToInt((areaMax.y - areaMin.y) / cellSize);
        visited = new bool[cellsX, cellsZ];
        blocked = new bool[cellsX, cellsZ];
        totalCount = cellsX * cellsZ;
        visitedCount = 0;
        blockedCount = 0;
        reachableCount = totalCount;  // Assume all reachable until scan
        lastCoverageFraction = 0f;
        
        Debug.Log($"[MapCoverageManager] Grid initialized: {cellsX}x{cellsZ} = {totalCount} cells");
    }
    
    /// <summary>
    /// Scans the environment to detect which cells are blocked by obstacles.
    /// Uses collider bounds instead of raycasting (more reliable).
    /// </summary>
    private void ScanForObstacles()
    {
        // Clear previous blocked status
        for (int x = 0; x < cellsX; x++)
            for (int z = 0; z < cellsZ; z++)
                blocked[x, z] = false;
        
        blockedCount = 0;
        HashSet<string> detectedObjects = new HashSet<string>();
        
        // Find all colliders in the scene
        Collider[] allColliders = FindObjectsOfType<Collider>();
        
        foreach (Collider col in allColliders)
        {
            // Skip triggers
            if (col.isTrigger) continue;
            
            // FIRST: Check if this should be excluded (robots, floors, ceilings)
            if (ShouldExcludeFromObstacles(col))
                continue;
            
            // Check if this object is an obstacle
            bool isObstacle = IsObstacle(col);
            
            if (!isObstacle) continue;
            
            string objName = col.gameObject.name;
            string objTag = col.tag;
            
            // Get the bounds of this obstacle and mark cells
            Bounds bounds = col.bounds;
            
            // Find all cells that overlap with this obstacle's footprint
            int minCellX = Mathf.FloorToInt((bounds.min.x - areaMin.x) / cellSize);
            int maxCellX = Mathf.CeilToInt((bounds.max.x - areaMin.x) / cellSize);
            int minCellZ = Mathf.FloorToInt((bounds.min.z - areaMin.y) / cellSize);
            int maxCellZ = Mathf.CeilToInt((bounds.max.z - areaMin.y) / cellSize);
            
            // Clamp to grid bounds
            minCellX = Mathf.Max(0, minCellX);
            maxCellX = Mathf.Min(cellsX - 1, maxCellX);
            minCellZ = Mathf.Max(0, minCellZ);
            maxCellZ = Mathf.Min(cellsZ - 1, maxCellZ);
            
            // Mark cells as blocked
            for (int x = minCellX; x <= maxCellX; x++)
            {
                for (int z = minCellZ; z <= maxCellZ; z++)
                {
                    if (!blocked[x, z])
                    {
                        blocked[x, z] = true;
                        blockedCount++;
                    }
                }
            }
            
            detectedObjects.Add($"{objName} (tag:{objTag})");
        }
        
        reachableCount = totalCount - blockedCount;
        
        float blockedPercent = (blockedCount / (float)totalCount) * 100f;
        Debug.Log($"[MapCoverageManager] Obstacle scan complete: " +
                  $"{blockedCount} blocked cells ({blockedPercent:F1}%), " +
                  $"{reachableCount} reachable cells");
        
        if (detectedObjects.Count > 0)
        {
            Debug.Log($"[MapCoverageManager] Detected {detectedObjects.Count} obstacle types: " +
                      $"{string.Join(", ", detectedObjects)}");
        }
        else
        {
            Debug.LogWarning($"[MapCoverageManager] ⚠️ NO obstacles detected!\n" +
                           $"  - Check that obstacles have tags: {string.Join(", ", obstacleTags)}\n" +
                           $"  - Or match name patterns: {string.Join(", ", obstacleNamePatterns)}");
        }
    }
    
    /// <summary>
    /// Check if a collider should be EXCLUDED from obstacle detection.
    /// Returns true for robots, floors, ceilings, etc.
    /// </summary>
    private bool ShouldExcludeFromObstacles(Collider col)
    {
        string objName = col.gameObject.name;
        
        // Check exclude patterns (name-based)
        foreach (string pattern in excludeNamePatterns)
        {
            if (objName.IndexOf(pattern, System.StringComparison.OrdinalIgnoreCase) >= 0)
                return true;
        }
        
        // Check if it's part of a robot by looking for ArticulationBody (URDF robots use this)
        // and checking if parent hierarchy contains "robot" in the name
        ArticulationBody artBody = col.GetComponentInParent<ArticulationBody>();
        if (artBody != null)
        {
            // Check if any parent is named "robot1", "robot2", etc.
            Transform t = col.transform;
            while (t != null)
            {
                if (t.name.ToLower().Contains("robot"))
                    return true;
                t = t.parent;
            }
        }
        
        // Check if tagged as Robot, Floor, or Ceiling
        if (SafeCompareTag(col.gameObject, "Robot")) return true;
        if (SafeCompareTag(col.gameObject, "Floor")) return true;
        if (SafeCompareTag(col.gameObject, "Ceiling")) return true;
        
        // Also check parent tags
        Transform parent = col.transform.parent;
        while (parent != null)
        {
            if (SafeCompareTag(parent.gameObject, "Robot")) return true;
            parent = parent.parent;
        }
        
        return false;
    }
    
    /// <summary>
    /// Check if a collider is an obstacle (by tag or name pattern).
    /// Uses safe tag comparison that doesn't throw if tag doesn't exist.
    /// </summary>
    private bool IsObstacle(Collider col)
    {
        // Check by tag (with safe comparison)
        foreach (string obstacleTag in obstacleTags)
        {
            if (SafeCompareTag(col.gameObject, obstacleTag))
                return true;
            
            // Also check parents
            Transform parent = col.transform.parent;
            while (parent != null)
            {
                if (SafeCompareTag(parent.gameObject, obstacleTag))
                    return true;
                parent = parent.parent;
            }
        }
        
        // Check by name pattern
        string objName = col.gameObject.name;
        foreach (string pattern in obstacleNamePatterns)
        {
            if (objName.IndexOf(pattern, System.StringComparison.OrdinalIgnoreCase) >= 0)
                return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Safely compare tag without throwing exception if tag doesn't exist.
    /// </summary>
    private bool SafeCompareTag(GameObject obj, string tag)
    {
        try
        {
            return obj.CompareTag(tag);
        }
        catch
        {
            // Tag doesn't exist in project - just return false
            return false;
        }
    }
    
    // ========================================================================
    // PUBLIC API
    // ========================================================================
    
    /// <summary>
    /// Register that a robot visited a position. Returns true if NEW cell.
    /// Both robots call this; the SHARED grid tracks overall coverage.
    /// Blocked cells are ignored - they don't count as visits.
    /// </summary>
    public bool RegisterVisit(Vector3 worldPos)
    {
        if (!WorldToCell(worldPos, out int ix, out int iz))
            return false;
        
        // Skip if cell is blocked by obstacle
        if (blocked[ix, iz])
            return false;
        
        if (!visited[ix, iz])
        {
            visited[ix, iz] = true;
            visitedCount++;
            newCellsThisStep++;
            return true;  // NEW CELL!
        }
        return false;
    }
    
    /// <summary>
    /// Get the fraction of REACHABLE cells that have been explored (0 to 1).
    /// This excludes blocked cells, so 100% is actually achievable!
    /// </summary>
    public float GetCoverageFraction()
    {
        if (reachableCount == 0) return 0f;
        return (float)visitedCount / reachableCount;
    }
    
    /// <summary>
    /// Get the count of reachable (non-blocked) cells.
    /// </summary>
    public int GetReachableCells()
    {
        return reachableCount;
    }
    
    /// <summary>
    /// Get the count of blocked cells.
    /// </summary>
    public int GetBlockedCells()
    {
        return blockedCount;
    }
    
    /// <summary>
    /// Get how much coverage increased since last call to this method.
    /// Used for calculating shared team reward.
    /// </summary>
    public float GetAndResetCoverageDelta()
    {
        float current = GetCoverageFraction();
        float delta = current - lastCoverageFraction;
        lastCoverageFraction = current;
        return delta;
    }
    
    /// <summary>
    /// Get how many NEW cells were discovered this physics step.
    /// </summary>
    public int GetNewCellsThisStep()
    {
        return newCellsThisStep;
    }
    
    /// <summary>
    /// Get absolute count of visited cells.
    /// </summary>
    public int GetVisitedCount()
    {
        return visitedCount;
    }
    
    /// <summary>
    /// Get total number of cells.
    /// </summary>
    public int GetTotalCells()
    {
        return totalCount;
    }
    
    /// <summary>
    /// Check if a position has been visited before.
    /// Useful for observations: "is the area in front of me explored?"
    /// </summary>
    public bool IsPositionVisited(Vector3 worldPos)
    {
        if (!WorldToCell(worldPos, out int ix, out int iz))
            return true;  // Outside area counts as "explored"
        return visited[ix, iz];
    }
    
    /// <summary>
    /// Get local unexplored density around a position.
    /// Returns 0-1 where 1 = all nearby REACHABLE cells unexplored (good to go there!)
    /// Blocked cells are excluded from the calculation.
    /// </summary>
    public float GetLocalUnexploredDensity(Vector3 worldPos, float radius = 2f)
    {
        int unexplored = 0;
        int total = 0;
        
        int cellRadius = Mathf.CeilToInt(radius / cellSize);
        
        if (!WorldToCell(worldPos, out int centerX, out int centerZ))
            return 0f;
        
        for (int dx = -cellRadius; dx <= cellRadius; dx++)
        {
            for (int dz = -cellRadius; dz <= cellRadius; dz++)
            {
                int x = centerX + dx;
                int z = centerZ + dz;
                
                if (x >= 0 && x < cellsX && z >= 0 && z < cellsZ)
                {
                    // Skip blocked cells
                    if (blocked[x, z])
                        continue;
                    
                    total++;
                    if (!visited[x, z])
                        unexplored++;
                }
            }
        }
        
        if (total == 0) return 0f;
        return (float)unexplored / total;
    }
    
    /// <summary>
    /// Reset the coverage grid. Called at the start of each episode.
    /// Note: Blocked cells are NOT reset - obstacles don't move between episodes.
    /// </summary>
    public void ResetCoverage()
    {
        for (int x = 0; x < cellsX; x++)
        {
            for (int z = 0; z < cellsZ; z++)
            {
                // Only reset visited status, NOT blocked status
                visited[x, z] = false;
            }
        }
        
        visitedCount = 0;
        lastCoverageFraction = 0f;
        newCellsThisStep = 0;
        
        Debug.Log("[MapCoverageManager] Coverage reset for new episode");
    }
    
    /// <summary>
    /// Force a re-scan of obstacles. Call this if obstacles move.
    /// </summary>
    public void RescanObstacles()
    {
        // Clear blocked status
        for (int x = 0; x < cellsX; x++)
            for (int z = 0; z < cellsZ; z++)
                blocked[x, z] = false;
        
        // Re-scan
        if (scanForObstacles)
        {
            ScanForObstacles();
        }
    }
    
    // ========================================================================
    // HELPER METHODS
    // ========================================================================
    
    private bool WorldToCell(Vector3 pos, out int ix, out int iz)
    {
        ix = Mathf.FloorToInt((pos.x - areaMin.x) / cellSize);
        iz = Mathf.FloorToInt((pos.z - areaMin.y) / cellSize);
        return ix >= 0 && ix < cellsX && iz >= 0 && iz < cellsZ;
    }
    
    private Vector3 CellToWorld(int ix, int iz)
    {
        return new Vector3(
            areaMin.x + (ix + 0.5f) * cellSize,
            0.1f,
            areaMin.y + (iz + 0.5f) * cellSize
        );
    }
    
    // ========================================================================
    // DEBUG VISUALIZATION
    // ========================================================================
    
    private void OnDrawGizmos()
    {
        if (!showDebugGrid || visited == null) return;
        
        for (int x = 0; x < cellsX; x++)
        {
            for (int z = 0; z < cellsZ; z++)
            {
                // Determine cell state
                bool isBlocked = blocked != null && blocked[x, z];
                bool isVisited = visited[x, z];
                
                // Skip cells based on view mode
                if (showOnlyVisitedCells && !isVisited && !isBlocked)
                    continue;
                
                Vector3 center = CellToWorld(x, z);
                
                // Color based on state: blocked (gray) > visited (green) > unvisited (red)
                if (isBlocked)
                    Gizmos.color = blockedColor;
                else if (isVisited)
                    Gizmos.color = visitedColor;
                else
                    Gizmos.color = unvisitedColor;
                
                Gizmos.DrawCube(center, new Vector3(cellSize * 0.9f, 0.02f, cellSize * 0.9f));
            }
        }
        
        // Boundary
        Gizmos.color = Color.yellow;
        Vector3 boundCenter = new Vector3((areaMin.x + areaMax.x) / 2f, 0.1f, (areaMin.y + areaMax.y) / 2f);
        Vector3 boundSize = new Vector3(areaMax.x - areaMin.x, 0.1f, areaMax.y - areaMin.y);
        Gizmos.DrawWireCube(boundCenter, boundSize);
    }
    
    private void OnGUI()
    {
        if (!showDebugGrid) return;
        
        float coverage = GetCoverageFraction() * 100f;
        
        // Show more detailed stats
        string text = $"Coverage: {visitedCount}/{reachableCount} reachable ({coverage:F1}%)";
        if (blockedCount > 0)
        {
            text += $" | {blockedCount} blocked cells";
        }
        
        GUI.color = Color.white;
        GUI.Label(new Rect(10, 10, 400, 25), text);
    }
}
