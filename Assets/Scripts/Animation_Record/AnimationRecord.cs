using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AnimationRecord : MonoBehaviour
{
    [Range(1,30)] public int decisionFrequency=5;
    public Transform Root;
    public Transform OrientationRef;
    public Transform[] BodyParts;
    public int numFeaturesPerBodyPart = 10;

    public Animator anim;
    public string filename;

    private int counter = 1;
    private int step = 0;
    private DataToText dataRecorder;
    private bool isSaved = false;

    private List<Vector3> velocities;
    private List<Vector3> prevPos;


    // Start is called before the first frame update
    void Awake()
    {
        int numFeatures = BodyParts.Length * numFeaturesPerBodyPart;
        dataRecorder = new DataToText(numFeatures);

        velocities = new List<Vector3>();
        prevPos = new List<Vector3>();
        for (int i = 0; i < BodyParts.Length; i++)
        {
            velocities.Add(Vector3.zero);
            prevPos.Add(BodyParts[i].transform.position);
        }
    }

    private void FixedUpdate()
    {
        counter += 1;
        
        OrientationRef.position = BodyParts[0].position;
        OrientationRef.forward = BodyParts[0].right;
        Vector3 temp = new Vector3(OrientationRef.forward.x, 0f, OrientationRef.forward.z);
        OrientationRef.rotation = Quaternion.FromToRotation(OrientationRef.forward, temp) * OrientationRef.rotation;

        ComputeVelocity();

        if (counter % decisionFrequency == 0 && !isSaved)
        {
            counter = 0;
            step += 1;

            float[] line = new float[numFeaturesPerBodyPart * BodyParts.Length];
            int ind = 0;
            int bpind = 0;

            foreach(Transform bp in BodyParts)
            {
                Quaternion rotations;
                if (bpind == 0)
                {
                    // if root rotation, do not pass the local rotation, pass the tilt instead
                    //rotations = Quaternion.Euler(new Vector3(bp.transform.eulerAngles.x, 0f, bp.transform.eulerAngles.z));
                    rotations = bp.transform.localRotation;
                }
                else
                {
                    rotations = bp.transform.localRotation;
                }
                Vector3 positions = OrientationRef.transform.InverseTransformPoint(bp.transform.position);
                Vector3 velocity = velocities[bpind];

                line[ind + 0] = positions.x;
                line[ind + 1] = positions.y;
                line[ind + 2] = positions.z;
                     
                line[ind + 3] = rotations.w;
                line[ind + 4] = rotations.x;
                line[ind + 5] = rotations.y;
                line[ind + 6] = rotations.z;

                line[ind + 7] = velocities[bpind].x;
                line[ind + 8] = velocities[bpind].y;
                line[ind + 9] = velocities[bpind].z;

                ind += numFeaturesPerBodyPart;
                bpind++;
            }

            dataRecorder.AddLine(line);

            if (step % 50 == 0)
            {
                Debug.Log("step" + step);
            }
        }

        var info = anim.GetCurrentAnimatorClipInfo(0);

        if (anim.GetCurrentAnimatorStateInfo(0).IsName("End") && !isSaved){
            dataRecorder.WriteToText(filename);
            isSaved = true;
        }
    }


    private void ComputeVelocity()
    {
        int ind = 0;
        foreach (var bodyPart in BodyParts)
        {
            Vector3 temp = OrientationRef.InverseTransformDirection((bodyPart.transform.position - prevPos[ind]) / Time.fixedDeltaTime);
            velocities[ind] = (velocities[ind] + temp)/2;
            prevPos[ind] = bodyPart.transform.position;
            ind++;
        }
    }

    private void ComputeVelocityAverage()
    {

    }
}
