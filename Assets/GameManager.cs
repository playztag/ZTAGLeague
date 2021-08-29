using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    // diameter of the area where the agent and flowers can be 
    // used for observing relative distance from agent to flower
    public const float ArenaSize = 20f;

    // A lookup dictionary of colliders to find the drone that it's attached to
    private Dictionary<Collider, Drone> sensorDroneDictionary;

    /// <summary>
    /// The list of all Drones in the arena
    /// </summary>
    public List<Drone> Drones { get; private set; }

    /// <summary>
    /// Called when area wakes up
    /// </summary>
    private void Awake()
    {
        // Initialize varibles
        sensorDroneDictionary = new Dictionary<Collider, Drone>();
        Drones = new List<Drone>();
    }

    private void Start()
    {
        // Find all flowers that are children of this GameObject/Transform
        FindChildDrones(transform);
    }

    /// <summary>
    /// Recursively finds all dronesthat are children of a parent transform
    /// </summary>
    /// <param name="parent">The parent of the children to check</param>
    private void FindChildDrones(Transform parent)
    {
        // found a single one

        for (int i = 0; i < parent.childCount; i++)
        {
            Transform child = parent.GetChild(i);




            if (child.CompareTag("Drone"))
            {
                // found drone
                Drones.Add(child.GetComponent<Drone>());

            }

            // look for more (for each)
            else
            {
                FindChildDrones(child);
            }
        }
    }

}