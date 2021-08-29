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
    [SerializeField] Movement otherDroneMovementScript;
    [SerializeField] GameObject goal;

    public bool isFrozen = false;
    public Environment env;

    public float taggingDistance = 20f;
    public float taggingAngle = 60f;

    [Tooltip("Transform of IR Beam")]
    public Transform IrBeam;

    [Tooltip("My rear light")]
    public GameObject RearLight;

    public Material unTaggedMaterial, taggedMaterial;

    public DroneBody otherDrone;


    private RayPerceptionSensorComponent3D rayHigh, rayMid, rayLow;
    private Rigidbody rb;

    float horizontalDistance = 5f;
    float verticalDistance = 5f;
    float maxStepDistance = 0f;

    private DroneBody db;
    


    public override void OnEpisodeBegin()
    {
        ////       Debug.Log("begin!");
        bool inFrontOfOtherDrone = true;
       // inFrontOfOtherDrone = Random.value > 0.5f;

        MoveToSafeRandomPosition(inFrontOfOtherDrone);

        rb = GetComponent<Rigidbody>();
        rb.velocity = Vector3.zero;

    }

    /// <summary>
    /// move the agent to a safe random position that doesn't collide with anything
    /// if in front of flower, also point the beak at the flower
    /// </summary>
    /// <param name="inFrontOfOtherDrone">whether to choose a spot in front of a flower</param>
    private void MoveToSafeRandomPosition(bool inFrontOfOtherDrone)
    {
        bool safePositionFound = false;
        int attemptsRemaining = 100; // prevent an infinite loop
        Vector3 potentialPosition = Vector3.zero;
        Quaternion potentialRotation = new Quaternion();

        // Loop until a safe position is found or we run out of attempts
        while (!safePositionFound && attemptsRemaining > 0)
        {
            attemptsRemaining--;
            if (inFrontOfOtherDrone)
            {
                //Pick a random flower
                ;

                // Position 10 to 20 cm in front of flower
                float distanceFromOtherDrone = UnityEngine.Random.Range(5f, 15f);
                potentialPosition = otherDrone.transform.position - otherDrone.transform.right * distanceFromOtherDrone;

                // Point beak at flower (bird's head is center of transform)
                Vector3 toDrone = otherDrone.transform.position - potentialPosition;
                Debug.DrawRay(transform.position, toDrone,Color.yellow);
                potentialRotation = Quaternion.LookRotation(toDrone, transform.forward);


            }
            else
            {
                // pick a random height from the ground
                float height = UnityEngine.Random.Range(5f, 45f);
                // pick a random radius from center of area
                float radius = UnityEngine.Random.Range(2f, 90f);
                //pick a random direction rotated from the Y axis;
                Quaternion direction = Quaternion.Euler(0f, UnityEngine.Random.Range(-180f, 180f), 0f);

                // Combine height, radius and direction to pick a potential position
                
                potentialPosition = env.transform.position + Vector3.up * height + direction * Vector3.forward * radius;

                // Choose and set random starting yaw                
                float yaw = UnityEngine.Random.Range(-180f, 180f);
                potentialRotation = Quaternion.Euler(0f, yaw, 0f);
            }

            // check to see if the agent will collide with anything
            Collider[] colliders = Physics.OverlapSphere(potentialPosition, 0.05f);

            // Safe position has been found if no colliders overlapped
            safePositionFound = colliders.Length == 0;
        }

        Debug.Assert(safePositionFound, "Could not find a safe position to spawn!");

        // Set the position and rotation
        transform.position = potentialPosition;
        transform.rotation = potentialRotation;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // observe the agents local rotation (4 observations)
        sensor.AddObservation(transform.localRotation.normalized);

        // get a vector from the beak tip to the nearest drone
        Vector3 toDrone = otherDrone.transform.position - transform.position;

        // observe a normalized vector pointing to the nearest drone (3 observations)
        sensor.AddObservation(toDrone.normalized);

        // observe if my drone is pointing at other drone (1 obs)
        Vector3 toOtherDrone = otherDrone.transform.position - IrBeam.transform.position;
        sensor.AddObservation(Vector3.Dot(toOtherDrone.normalized, otherDrone.transform.right.normalized));

        // observe if my drone is facing same direction as other drone (1 obs)
        sensor.AddObservation(Vector3.Dot(transform.right.normalized, otherDrone.transform.right.normalized));

        // observe distance to other drone
        sensor.AddObservation(Vector3.Distance(transform.position, otherDrone.transform.position)/50f);
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
            //DroneMovementObject.W = actions.DiscreteActions[0] == 1 ? true : false;
            //DroneMovementObject.S = actions.DiscreteActions[0] == 2 ? true : false;

            //DroneMovementObject.A = actions.DiscreteActions[1] == 1 ? true : false;
            //DroneMovementObject.D = actions.DiscreteActions[1] == 2 ? true : false;

            //DroneMovementObject.I = actions.DiscreteActions[2] == 1 ? true : false;
            //DroneMovementObject.K = actions.DiscreteActions[2] == 2 ? true : false;

            //DroneMovementObject.J = actions.DiscreteActions[3] == 1 ? true : false;
            //DroneMovementObject.L = actions.DiscreteActions[3] == 2 ? true : false;

            DroneMovementObject.customFeed_forward = -actions.ContinuousActions[0];
            DroneMovementObject.customFeed_backward = actions.ContinuousActions[0];

            DroneMovementObject.customFeed_leftward = -actions.ContinuousActions[1];
            DroneMovementObject.customFeed_rightward = actions.ContinuousActions[1];

            DroneMovementObject.customFeed_upward = actions.ContinuousActions[2];
            DroneMovementObject.customFeed_downward = -actions.ContinuousActions[2];

            DroneMovementObject.customFeed_rotateLeft = -actions.ContinuousActions[3];
            DroneMovementObject.customFeed_rotateRight = actions.ContinuousActions[3];

        }

        //AddReward(-1f / MaxStep); // motivated to speed up search

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
        if (other.collider.CompareTag("Boundary") || other.collider.CompareTag("Obstacle"))
        {
            Debug.Log("Physical collision occured");
            AddReward(-.5f);
        }
    }

    private void OnTriggerEnter(Collider other)
    {       
        if (other.CompareTag("Drone"))
        {
            Debug.Log("trigged!!! Drone" + other.tag);
            Vector3 toOtherDrone = other.transform.position - IrBeam.transform.position;
            if (Vector3.Angle(toOtherDrone.normalized, other.transform.right.normalized) < taggingAngle)
            {
                Debug.Log("HIT!!!");
                other.GetComponent<DroneBody>().setLights();
                otherDroneMovementScript.increaseDifficulty();
                otherDroneMovementScript.chooseRandomWayPoint();
            }
        }
    }
    
    
    private void OnTriggerStay(Collider other)

    {
        if (other.CompareTag("Drone"))
        {
            Vector3 toOtherDrone = other.transform.position - IrBeam.transform.position;
            //Debug.Log("dotproduct: " + Vector3.Dot(toOtherDrone.normalized, other.transform.right.normalized));
            //Debug.Log("angle between: " + Vector3.Angle(toOtherDrone.normalized, other.transform.right.normalized));
            Debug.DrawRay(transform.localPosition, toOtherDrone, Color.green);
            Debug.DrawRay(other.transform.localPosition, other.transform.right.normalized, Color.red);
            if (Vector3.Angle(toOtherDrone.normalized, other.transform.right.normalized) < taggingAngle)
            {

                Debug.Log("HIT CREDIT!!!");
                AddReward(.1f);
                otherDroneMovementScript.chooseRandomWayPoint();
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

    private void Update()
    {
        if(Vector3.Distance(otherDrone.transform.position, transform.position) < taggingDistance)
        {
            Debug.Log("I'm close!!!");
            AddReward(0.01f);
        }

        Debug.DrawRay(IrBeam.transform.position, otherDrone.transform.position - IrBeam.transform.position, Color.red);
        
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Drone"))
        {
            Vector3 toOtherDrone = other.transform.position - IrBeam.transform.position;
            //Debug.Log("dotproduct: " + Vector3.Dot(toOtherDrone.normalized, other.transform.right.normalized));
            //Debug.Log("angle between: " + Vector3.Angle(toOtherDrone.normalized, other.transform.right.normalized));


            other.GetComponent<DroneBody>().unsetLights();


        }

    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Right_Y");
        continuousActionsOut[1] = Input.GetAxis("Right_X");
        continuousActionsOut[2] = Input.GetAxis("Left_Y");
        continuousActionsOut[3] = Input.GetAxis("Left_X");

        //var discreteActions = actionsOut.DiscreteActions;

        //pitch
        //if (Input.GetKey(KeyCode.Z))
        //{

        //    discreteActions[0] = 1;
        //}
        //else if (Input.GetKey(KeyCode.X))
        //{
        //    discreteActions[0] = 2;
        //}
        //else
        //{
        //    discreteActions[0] = 0;
        //}

        //if(Input.GetAxis("Right_Y") > 0.2)
        //{
        //    discreteActions[0] = 2;
        //}
        //else if (Input.GetAxis("Right_Y") < -0.2)
        //{
        //    discreteActions[0] = 1;
        //}
        //else
        //{
        //    discreteActions[0] = 0;
        //}

        ////roll
        //if (Input.GetKey(KeyCode.C))
        //{

        //    discreteActions[1] = 1;
        //}
        //else if (Input.GetKey(KeyCode.V))
        //{
        //    discreteActions[1] = 2;
        //}
        //else
        //{
        //    discreteActions[1] = 0;
        //}


        //if (Input.GetAxis("Right_X") > 0.2)
        //{
        //    discreteActions[1] = 2;
        //}
        //else if (Input.GetAxis("Right_X") < -0.2)
        //{
        //    discreteActions[1] = 1;
        //}
        //else
        //{
        //    discreteActions[1] = 0;
        //}


        ////thrust
        //if (Input.GetKey(KeyCode.B))
        //{

        //    discreteActions[2] = 1;
        //}
        //else if (Input.GetKey(KeyCode.N))
        //{
        //    discreteActions[2] = 2;
        //}
        //else
        //{
        //    discreteActions[2] = 0;
        //}

        //if (Input.GetAxis("Left_Y") > 0.2)
        //{
        //    discreteActions[2] = 1;
        //}
        //else if (Input.GetAxis("Left_Y") < -0.2)
        //{
        //    discreteActions[2] = 2;
        //}
        //else
        //{
        //    discreteActions[2] = 0;
        //}

        ////yaw
        //if (Input.GetKey(KeyCode.M))
        //{

        //    discreteActions[3] = 1;
        //}
        //else if (Input.GetKey(KeyCode.Comma))
        //{
        //    discreteActions[3] = 2;
        //}
        //else
        //{
        //    discreteActions[3] = 0;
        //}

        //if (Input.GetAxis("Left_X") > 0.2)
        //{
        //    discreteActions[3] = 2;
        //}
        //else if (Input.GetAxis("Left_X") < -0.2)
        //{
        //    discreteActions[3] = 1;
        //}
        //else
        //{
        //    discreteActions[3] = 0;
        //}


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