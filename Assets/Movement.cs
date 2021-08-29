using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Movement : MonoBehaviour
{
    public GameObject Environment;
    [SerializeField]
    DroneMovement DroneMovementObject;
    private float difficultyDistance = 10f;
 
    Vector3 location;

    private Vector3 currentWaypoint;
    // Start is called before the first frame update
    void Start()
    {
        location = transform.position;
        chooseRandomWayPoint();
    }

    public void increaseDifficulty()
    {
        difficultyDistance++;
        Mathf.Clamp( difficultyDistance, 10, 100);
        Debug.Log("Difficulty inreased");
    }
    public void chooseRandomWayPoint()
    {
        currentWaypoint = Environment.transform.position+ new Vector3(Random.Range(-difficultyDistance, difficultyDistance),Random.Range(25-difficultyDistance/5f, 25+difficultyDistance/5f), Random.Range(-difficultyDistance, difficultyDistance));
    }

    bool isCloseToWayPoint(Vector3 waypoint)
    {
        if (Vector3.Distance(transform.position, waypoint) < 5f) return true;
        else return false;
    }

    // Update is called once per frame
    void Update()
    {
        //transform.position = location + new Vector3(
        //    Mathf.Sin(Time.fixedTime*0.54f) * 45f, 
        //    Mathf.Sin(Time.fixedTime*0.3f) * 25f+20f, 
        //    Mathf.Cos(Time.fixedTime * 0.48f) * 10f);

        //transform.Rotate(transform.up, Mathf.Cos(Time.fixedTime/10f));

        // match drone height with waypoint
        if(currentWaypoint.y > transform.position.y)
        {
            DroneMovementObject.I = true;
            DroneMovementObject.K = false;
        }
        else
        {
            DroneMovementObject.K = true;
            DroneMovementObject.I = false;
        }

        // take cross product of TOwaypoint vector and my own forward vector to see if I need to rotate
        Vector3 toWayPoint = currentWaypoint - transform.position;
        Debug.DrawRay(transform.position, toWayPoint, Color.blue);

        if(Vector3.Cross(toWayPoint, transform.forward).y > 0)
        {
            DroneMovementObject.J = true;
            DroneMovementObject.L = false;
        }
        else
        {
            DroneMovementObject.L = true;
            DroneMovementObject.J = false;
        }

        // always go forward
        DroneMovementObject.W = true;

        if (isCloseToWayPoint(currentWaypoint))
        {
            chooseRandomWayPoint();
        }      
    }
}
