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
    
    [Tooltip("Height to cast rays from (should be above obstacles)")]
    public float scanHeight = 5f;
    
    [Tooltip("Layers that count as obstacles (block cells)")]
    public LayerMask obstacleLayerMask = -1;
    
    [Tooltip("Tags to IGNORE when scanning (e.g. Floor, Robot)")]
    public string[] ignoreTags = { "Floor", "Robot", "Untagged" };
    
    [Tooltip("Click to rescan obstacles (useful after moving things)")]
    [ContextMenuItem("Rescan Obstacles Now", "RescanObstacles")]
    public bool rescanButton = false;  // Dummy field for context menu
    
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
        
        InitializeGrid();
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
    }
    
    void FixedUpdate()
    {
        // Reset new cells counter each physics step
        newCellsThisStep = 0;
    }
    
    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    
    private void InitializeGrid()
    {
        cellsX = Mathf.CeilToInt((areaMax.x - areaMin.x) / cellSize);
        cellsZ = Mathf.CeilToInt((areaMax.y - areaMin.y) / cellSize);
        visited = new bool[cellsX, cellsZ];
        blocked = new bool[cellsX, cellsZ];
        totalCount = cellsX * cellsZ;
        visitedCount = 0;
        blockedCount = 0;
        lastCoverageFraction = 0f;
        
        Debug.Log($"[MapCoverageManager] Grid initialized: {cellsX}x{cellsZ} = {totalCount} cells");
        
        // Scan for obstacles if enabled
        if (scanForObstacles)
        {
            ScanForObstacles();
        }
        else
        {
            reachableCount = totalCount;
        }
    }
    
    /// <summary>
    /// Scans the environment from above to detect which cells are blocked by obstacles.
    /// Blocked cells are marked and excluded from the coverage calculation.
    /// </summary>
    private void ScanForObstacles()
    {
        blockedCount = 0;
        
        for (int x = 0; x < cellsX; x++)
        {
            for (int z = 0; z < cellsZ; z++)
            {
                Vector3 cellCenter = CellToWorld(x, z);
                Vector3 rayOrigin = cellCenter + Vector3.up * scanHeight;
                
                // Cast ray downward to check for obstacles
                if (Physics.Raycast(rayOrigin, Vector3.down, out RaycastHit hit, scanHeight + 1f, obstacleLayerMask))
                {
                    // Check if the hit object should be ignored
                    bool shouldIgnore = false;
                    
                    // Check tag
                    foreach (string ignoreTag in ignoreTags)
                    {
                        if (hit.collider.CompareTag(ignoreTag))
                        {
                            shouldIgnore = true;
                            break;
                        }
                    }
                    
                    // Also check parent tags (for URDF hierarchies)
                    if (!shouldIgnore)
                    {
                        Transform parent = hit.collider.transform.parent;
                        while (parent != null && !shouldIgnore)
                        {
                            foreach (string ignoreTag in ignoreTags)
                            {
                                if (parent.CompareTag(ignoreTag))
                                {
                                    shouldIgnore = true;
                                    break;
                                }
                            }
                            parent = parent.parent;
                        }
                    }
                    
                    // Check if it's a floor/ground by name (backup check)
                    string nameLower = hit.collider.name.ToLower();
                    if (nameLower.Contains("floor") || nameLower.Contains("ground") || 
                        nameLower.Contains("plane"))
                    {
                        shouldIgnore = true;
                    }
                    
                    // If not ignored, this cell is blocked
                    if (!shouldIgnore)
                    {
                        blocked[x, z] = true;
                        blockedCount++;
                    }
                }
            }
        }
        
        reachableCount = totalCount - blockedCount;
        
        float blockedPercent = (blockedCount / (float)totalCount) * 100f;
        Debug.Log($"[MapCoverageManager] Obstacle scan complete: " +
                  $"{blockedCount} blocked cells ({blockedPercent:F1}%), " +
                  $"{reachableCount} reachable cells");
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
