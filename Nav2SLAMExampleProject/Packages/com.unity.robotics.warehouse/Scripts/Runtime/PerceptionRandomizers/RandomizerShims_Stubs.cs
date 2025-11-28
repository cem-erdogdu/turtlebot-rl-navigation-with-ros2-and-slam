using UnityEngine;
using UnityEngine.Perception.Randomization.Randomizers; // <- important!

// Minimal no-op shims so Warehouse compiles with Perception 1.x.
// These satisfy Scenario.GetRandomizer<T>() wher 

public class ShelfBoxRandomizerShim : Randomizer
{
    [SerializeField] private GameObject defaultBoxPrefab;

    // Warehouse code calls this method:
    public GameObject GetBoxPrefab()
    {
        if (defaultBoxPrefab != null) return defaultBoxPrefab;

        // Fallback: create a simple cube if no prefab is assigned.
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = "FallbackBoxPrefab";
        return go;
    }

    // No behavior needed; Randomizer methods are virtual.
    // Keeping class empty is fine for our stub.
}

public class FloorBoxRandomizerShim : Randomizer { }
public class LocalRotationRandomizerShim : Randomizer { }
