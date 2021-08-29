using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RearLight : MonoBehaviour

   
{

    public Material HitMaterial;
    public Material UnhitMaterial;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    public void setHitColor()
    {
        Renderer rend = transform.GetComponent<Renderer>();
        rend.material = HitMaterial;
    }

    public void unSetHitColor()
    {
        Renderer rend = transform.GetComponent<Renderer>();
        rend.material = HitMaterial;
    }

        // Update is called once per frame
        void Update()
    {
        
    }
}
