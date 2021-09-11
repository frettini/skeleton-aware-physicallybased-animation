using System.IO;
using System.Text;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class SaveInitInfo : MonoBehaviour
{

    public LaFanLegs agent;

    // Start is called before the first frame update
    void Start()
    {
        // generate initial message to send
        List<List<float>> initRotations = new List<List<float>>();
        List<List<float>> initPositions = new List<List<float>>();

        List<int> parents = GenerateParentList();

        float frametime = agent.gameObject.GetComponent<DecisionRequester>().DecisionPeriod * Time.fixedDeltaTime;
        Debug.Log(Time.fixedDeltaTime);
        Debug.Log(parents);

        string txt_rot = "";
        string txt_pos = "";
        string txt_parent = "";
        int count = 0;

        foreach (var bodyPart in agent.m_JdController.bodyPartsList)
        {
            List<float> rot = new List<float>();
            List<float> pos = new List<float>();

            rot.Add(bodyPart.startingLocalRot.w);
            rot.Add(bodyPart.startingLocalRot.x);
            rot.Add(bodyPart.startingLocalRot.y);
            rot.Add(bodyPart.startingLocalRot.z);

            pos.Add(bodyPart.startingLocalPos.x);
            pos.Add(bodyPart.startingLocalPos.y);
            pos.Add(bodyPart.startingLocalPos.z);

            initRotations.Add(rot);
            initPositions.Add(pos);

            txt_pos += pos[0].ToString() + ",";
            txt_pos += pos[1].ToString() + ",";
            txt_pos += pos[2].ToString();

            txt_rot += rot[0].ToString() + ",";
            txt_rot += rot[1].ToString() + ",";
            txt_rot += rot[2].ToString() + ",";
            txt_rot += rot[3].ToString();

            if(count == parents.Count - 1)
            {
                txt_pos += '\n';
                txt_rot += '\n';
            }
            else
            {
                txt_pos += ',';
                txt_rot += ',';
            }

            count++;
        }


        for(int i = 0; i<parents.Count; i++)
        {
            txt_parent += parents[i].ToString();
            if (i == parents.Count - 1)
            {
                txt_parent += '\n';
            }
            else
            {
                txt_parent += ',';
            }

        }

        // save info
        var text = new StringBuilder();
        text.Append(parents.Count.ToString() + "\n"); // num_joints
        text.Append(txt_pos); // init positions
        text.Append(txt_rot); // init rotations
        text.Append(txt_parent); // parent list
        text.Append(frametime); // frametime

        //after your loop
        File.WriteAllText(Application.dataPath + "/init_sk_info.txt", text.ToString());
        Debug.Log("Finished Saving Data!");

    }

    List<int> GenerateParentList()
    {
        int base_parent = -1;
        List<int> parents_list = new List<int>();
        int node_count = -1;

        void dfs(Transform root, int parent, int node_id, List<int> parents)
        {
            parents.Add(parent);
            Debug.Log(parent);

            foreach (Transform child in root.transform)
            {
                if (child.GetComponent<Collider>() != null)
                {
                    dfs(child, node_id, ++node_count, parents);
                }
            }
        }

        dfs(agent.hips, base_parent, ++node_count, parents_list);

        return parents_list;
    }
}
