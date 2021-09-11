using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents.SideChannels;
using Unity.MLAgents;
using Unity.MLAgentsExamples;

public class MySideChannelManager : MonoBehaviour
{
    // Lafan agent
    public LaFanLegs agent;

    private SkeletonSideChannel skeletonSideChannel;

    // Start is called before the first frame update
    void Start()
    {

        skeletonSideChannel = new SkeletonSideChannel();
        SideChannelManager.RegisterSideChannel(skeletonSideChannel);

        // generate initial message to send
        List<List<float>> initRotations = new List<List<float>>();
        List<List<float>> initPositions = new List<List<float>>();
        
        List<int> parents = GenerateParentList();

        float frametime = agent.gameObject.GetComponent<DecisionRequester>().DecisionPeriod * Time.fixedDeltaTime;
        Debug.Log(Time.fixedDeltaTime);
        Debug.Log(parents);

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
        }

        skeletonSideChannel.SendSkeletonInfo(initRotations, initPositions, parents, frametime);

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
                if(child.GetComponent<GroundContact>() != null)
                {
                    dfs(child, node_id, ++node_count, parents);
                }
            }
        }

        dfs(agent.hips, base_parent, ++node_count, parents_list);

        return parents_list;
    }

    public void OnDestroy()
    {
        // De-register the Debug.Log callback
        if (Academy.IsInitialized)
        {
            SideChannelManager.UnregisterSideChannel(skeletonSideChannel);
        }
    }
}
