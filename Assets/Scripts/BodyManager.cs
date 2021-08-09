using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SKAMP
{
    public class BodyManager : MonoBehaviour
    {
     
        // Start is called before the first frame update
        void Awake()
        {

            IgnorePhysics();    
            
        }

        private void IgnorePhysics()
        {
            List<CapsuleCollider> colliderQueue = new List<CapsuleCollider>();
            colliderQueue.Add(GetComponent<CapsuleCollider>());

            while(colliderQueue.Count > 0)
            {
                // Get the next transform in queue, and check for child transforms
                CapsuleCollider currCollider = colliderQueue[0];
                colliderQueue.RemoveAt(0);

                // Loop through each children and check if their colliders overlap
                foreach (Transform child in currCollider.transform)
                {
                    CapsuleCollider childCollider = child.GetComponent<CapsuleCollider>();
                    
                    if (childCollider != null)
                    {
                        Collider[] cols = Physics.OverlapCapsule(childCollider.bounds.max, childCollider.bounds.min, childCollider.radius);
                        //Debug.Log("Child collider bound:" + childCollider.bounds.max.ToString() + " and  " + childCollider.bounds.min.ToString() + " " + childCollider.name);
                        if (cols != null)
                        {
                            foreach (Collider col in cols)
                            {
                                if (col.name == currCollider.name)
                                {
                                    //Debug.Log("Ignore collision between " + currCollider.name + " and " + childCollider.name);
                                    // If overlap is found, ignore the collision
                                    Physics.IgnoreCollision(childCollider, currCollider);
                                }
                            }
                        }

                        colliderQueue.Add(childCollider);
                    }
                }
            }
        }

    }
} 
