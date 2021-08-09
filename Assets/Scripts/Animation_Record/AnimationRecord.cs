using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AnimationRecord : MonoBehaviour
{
    [Range(1,30)] public int decisionFrequency=5;
    public Transform Root;
    public Transform[] BodyParts;
    public int numFeaturesPerBodyPart = 7;

    public Animator anim;

    private int counter = 1;
    private int step = 0;
    private DataToText dataRecorder;
    private bool isSaved = false;
    private bool hasStarted = false;

    // Start is called before the first frame update
    void Start()
    {
        int numFeatures = BodyParts.Length * numFeaturesPerBodyPart;
        dataRecorder = new DataToText(numFeatures);
    }

    private void FixedUpdate()
    {
        counter += 1;

        if(counter % decisionFrequency == 0 && !isSaved)
        {
            counter = 0;
            step += 1;

            float[] line = new float[numFeaturesPerBodyPart * BodyParts.Length];
            int ind = 0;

            foreach(Transform bp in BodyParts)
            {
                Vector3 positions = Root.transform.InverseTransformPoint(bp.transform.position);
                Quaternion rotations = bp.transform.localRotation;

                line[ind + 0] = positions.x;
                line[ind + 1] = positions.y;
                line[ind + 2] = positions.z;
                     
                line[ind + 3] = rotations.w;
                line[ind + 4] = rotations.x;
                line[ind + 5] = rotations.y;
                line[ind + 6] = rotations.z;

                ind += numFeaturesPerBodyPart;
            }

            dataRecorder.AddLine(line);

            if (step % 50 == 0)
            {
                Debug.Log("step" + step);
            }
        }

        var info = anim.GetCurrentAnimatorClipInfo(0);

        if(anim.GetCurrentAnimatorStateInfo(0).IsName("Walk") && !hasStarted){
            hasStarted = true;
        }

        if (!anim.GetCurrentAnimatorStateInfo(0).IsName("Walk") && !isSaved && hasStarted){
            dataRecorder.WriteToText("/walk1_subject1_csv.txt");
            isSaved = true;
        }
    }
}
