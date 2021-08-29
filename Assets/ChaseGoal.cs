using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;

public class ChaseGoal : Agent
{
    [SerializeField] DroneMovement DroneMovementObject;
    [SerializeField] GameObject goal;

    public bool isFrozen = false;


    [Tooltip("Transform of IR Beam")]
    public Transform IrBeam;

    [Tooltip("My rear light")]
    public GameObject RearLight;

    public Material unTaggedMaterial, taggedMaterial;   


    private RayPerceptionSensorComponent3D rayHigh, rayMid, rayLow;
    private Rigidbody rb;

    float horizontalDistance = 5f;
    float verticalDistance = 5f;
    float maxStepDistance = 0f;

    private DroneBody db;


    public override void OnEpisodeBegin()
    {
        ////       Debug.Log("begin!");

        ////put drone in a new location randomly
        //transform.localPosition = new Vector3(Random.Range(-horizontalDistance, horizontalDistance),
        //                                      Random.Range(0, verticalDistance),
        //                                      Random.Range(-horizontalDistance, horizontalDistance));

        ////put goal in a new location randomly
        //goal.transform.localPosition = new Vector3(Random.Range(-horizontalDistance, horizontalDistance),
        //                                   Random.Range(0, verticalDistance),
        //                                   Random.Range(-horizontalDistance, horizontalDistance));
        rb = GetComponent<Rigidbody>();
        rb.velocity = Vector3.zero;

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(rb.velocity);

    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        // float thrust = actions.ContinuousActions[0];
        // float roll = actions.ContinuousActions[1];
        // float pitch = actions.ContinuousActions[2];
        // float yaw = actions.ContinuousActions[3];

        // DroneMovementObject.Vertical_W = thrust;
        // DroneMovementObject.Vertical_S = thrust;

        // DroneMovementObject.Horizontal_J = roll;
        // DroneMovementObject.Horizontal_L = roll;

        // DroneMovementObject.Vertical_I = pitch;
        // DroneMovementObject.Vertical_K = pitch;

        // DroneMovementObject.Horizontal_A = yaw;
        // DroneMovementObject.Horizontal_D = yaw;

        // discrete movements of drone

        if (!isFrozen)
        {
            DroneMovementObject.W = actions.DiscreteActions[0] == 1 ? true : false;
            DroneMovementObject.S = actions.DiscreteActions[0] == 2 ? true : false;

            DroneMovementObject.A = actions.DiscreteActions[1] == 1 ? true : false;
            DroneMovementObject.D = actions.DiscreteActions[1] == 2 ? true : false;

            DroneMovementObject.I = actions.DiscreteActions[2] == 1 ? true : false;
            DroneMovementObject.K = actions.DiscreteActions[2] == 2 ? true : false;

            DroneMovementObject.J = actions.DiscreteActions[3] == 1 ? true : false;
            DroneMovementObject.L = actions.DiscreteActions[3] == 2 ? true : false;
        }

        AddReward(-1f / MaxStep); // motivated to speed up search

        // if (actions.DiscreteActions[0] == 1)
        // {
        //     Debug.Log("Moving Z!!!!");
        //     DroneMovementObject.W = true;
        //     DroneMovementObject.S = false;
        // }
        // else if (actions.DiscreteActions[0] == 2)
        // {
        //     Debug.Log("Moving X!!!!");
        //     DroneMovementObject.S = true;
        //     DroneMovementObject.W = false;
        // }
        // else
        // {
        //     DroneMovementObject.S = false;
        //     DroneMovementObject.W = false;
        // }
    }

    private float increaseWithMax(float num, float inc, float max)
    {
        num = (num < max) ? num + inc : max;
        return num;
    }

    private float decreaseWithMax(float num, float dec, float min)
    {
        num = (num > min) ? num - dec : min;
        return num;
    }

    private void OnCollisionEnter(Collision other)
    {
        //if (other.collider.tag == "Goal")
        //{
        //    AddReward(1);
        //    Debug.Log("GOAL!!!");
        //    horizontalDistance = increaseWithMax(horizontalDistance, 1, 100);
        //    verticalDistance = increaseWithMax(verticalDistance, 1, 50);
        //    MaxStep = (int)increaseWithMax((float)MaxStep, 10, 10000);
        //    EndEpisode();
        //}
        //else
        //{
        //    AddReward(-1);
        //    Debug.Log("crash!!!");
        //    horizontalDistance = decreaseWithMax(horizontalDistance, 1, 10);
        //    verticalDistance = decreaseWithMax(verticalDistance, 1, 10);
        //    EndEpisode();
        //}
        if (other.collider.CompareTag("Boundary") || other.collider.CompareTag("Obstacle"))
        {
            Debug.Log("Physical collision occured");
            AddReward(-.5f);
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.CompareTag("Drone"))
        {
            Vector3 toOtherDrone = other.transform.position - IrBeam.transform.position;
            //Debug.Log("dotproduct: " + Vector3.Dot(toOtherDrone.normalized, other.transform.right.normalized));
            //Debug.Log("angle between: " + Vector3.Angle(toOtherDrone.normalized, other.transform.right.normalized));
            Debug.DrawRay(transform.position, toOtherDrone, Color.green);
            Debug.DrawRay(other.transform.position, other.transform.right.normalized, Color.red);
            if (Vector3.Angle(toOtherDrone.normalized, other.transform.right.normalized) < 30f)
            {
                Debug.Log("HIT!!!" + other.gameObject.transform.childCount);
                //db = (DroneBody) other;
                other.GetComponent<DroneBody>().setLights();


            }
        }

        ////}
        //Debug.DrawRay(other.transform.position, transform.position, Color.red);

        //Debug.Log("dotproduct: " + Vector3.Dot(transform.forward.normalized, other.transform.forward.normalized));

        //if (other.CompareTag("Rear Collider B") && 
        //    Vector3.Dot(transform.forward.normalized, other.transform.forward.normalized)>.7)
        //{
        //    Debug.Log("Drone A tags B!!!");
        //    AddReward(.1f);           
        //}



        // if getting tagged by other drone
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Drone"))
        {
            Vector3 toOtherDrone = other.transform.position - IrBeam.transform.position;
            //Debug.Log("dotproduct: " + Vector3.Dot(toOtherDrone.normalized, other.transform.right.normalized));
            //Debug.Log("angle between: " + Vector3.Angle(toOtherDrone.normalized, other.transform.right.normalized));


            Debug.Log("HIT!!!" + other.gameObject.transform.childCount);
            other.GetComponent<DroneBody>().unsetLights();


        }

    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //var continuousActionsOut = actionsOut.ContinuousActions;
        var discreteActions = actionsOut.DiscreteActions;

        //pitch
        if (Input.GetKey(KeyCode.Z))
        {

            discreteActions[0] = 1;
        }
        else if (Input.GetKey(KeyCode.X))
        {
            discreteActions[0] = 2;
        }
        else
        {
            discreteActions[0] = 0;
        }

        if(Input.GetAxis("Right_Y") > 0.2)
        {
            discreteActions[0] = 2;
        }
        else if (Input.GetAxis("Right_Y") < -0.2)
        {
            discreteActions[0] = 1;
        }
        else
        {
            discreteActions[0] = 0;
        }

        //roll
        if (Input.GetKey(KeyCode.C))
        {

            discreteActions[1] = 1;
        }
        else if (Input.GetKey(KeyCode.V))
        {
            discreteActions[1] = 2;
        }
        else
        {
            discreteActions[1] = 0;
        }


        if (Input.GetAxis("Right_X") > 0.2)
        {
            discreteActions[1] = 2;
        }
        else if (Input.GetAxis("Right_X") < -0.2)
        {
            discreteActions[1] = 1;
        }
        else
        {
            discreteActions[1] = 0;
        }


        //thrust
        if (Input.GetKey(KeyCode.B))
        {

            discreteActions[2] = 1;
        }
        else if (Input.GetKey(KeyCode.N))
        {
            discreteActions[2] = 2;
        }
        else
        {
            discreteActions[2] = 0;
        }

        if (Input.GetAxis("Left_Y") > 0.2)
        {
            discreteActions[2] = 1;
        }
        else if (Input.GetAxis("Left_Y") < -0.2)
        {
            discreteActions[2] = 2;
        }
        else
        {
            discreteActions[2] = 0;
        }

        //yaw
        if (Input.GetKey(KeyCode.M))
        {

            discreteActions[3] = 1;
        }
        else if (Input.GetKey(KeyCode.Comma))
        {
            discreteActions[3] = 2;
        }
        else
        {
            discreteActions[3] = 0;
        }

        if (Input.GetAxis("Left_X") > 0.2)
        {
            discreteActions[3] = 2;
        }
        else if (Input.GetAxis("Left_X") < -0.2)
        {
            discreteActions[3] = 1;
        }
        else
        {
            discreteActions[3] = 0;
        }


        // //roll
        // if (Input.GetKey(KeyCode.C))
        // {
        //    // continuousActionsOut[1] = -1;
        // }
        // else if (Input.GetKey(KeyCode.V))
        // {
        //    // continuousActionsOut[1] = 1;
        // }
        // else
        // {
        //    // continuousActionsOut[1] = 0;
        // }

        // //pitch
        // if (Input.GetKey(KeyCode.B))
        // {
        //     //continuousActionsOut[2] = -1;
        // }
        // else if (Input.GetKey(KeyCode.N))
        // {
        //     //continuousActionsOut[2] = 1;
        // }
        // else
        // {
        //     //continuousActionsOut[2] = 0;
        // }

        // //yaw
        // if (Input.GetKey(KeyCode.M))
        // {
        //     //continuousActionsOut[3] = -1;
        // }
        // else if (Input.GetKey(KeyCode.Comma))
        // {
        //    // continuousActionsOut[3] = 1;
        // }
        // else
        // {
        //    // continuousActionsOut[3] = 0;
        // }


    }
}