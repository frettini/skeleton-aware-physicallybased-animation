using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents.SideChannels;
using Unity.MLAgents;

public class MySideChannelManager : MonoBehaviour
{
    // Lafan agent
    public LaFanLine agent; 

    private SkeletonSideChannel skeletonSideChannel;

    // Start is called before the first frame update
    void Start()
    {
        skeletonSideChannel = new SkeletonSideChannel();
        SideChannelManager.RegisterSideChannel(skeletonSideChannel);

        // generate initial message to send
        List<List<float>> initRotations = new List<List<float>>();
        List<List<float>> initPositions = new List<List<float>>();

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

        skeletonSideChannel.SendSkeletonInfo(initRotations, initPositions);
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
