using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OrientationObject : MonoBehaviour
{

    public Transform reference;


    // Update is called once per frame
    void Update()
    {


        transform.position = reference.position;
        transform.forward = reference.forward; //right;
        Vector3 temp = new Vector3(transform.forward.x, 0f, transform.forward.z);
        transform.rotation = Quaternion.FromToRotation(transform.forward, temp) * transform.rotation;
    }
}
