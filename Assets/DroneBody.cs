using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DroneBody : MonoBehaviour
{
    public GameObject droneBody;
    public RearLight lights;



    public Vector3 droneForwardVector
    {
        get
        {
            return droneBody.transform.forward;
        }
    }
    public void Start()
    {
       
    }

    public void setLights()
    {
        //lights = transform.GetComponent<RearLight>();  
        lights.setHitColor();
    }

    public void unsetLights()
    {
        //lights = transform.GetComponent<RearLight>();
        lights.unSetHitColor();

    }

}
