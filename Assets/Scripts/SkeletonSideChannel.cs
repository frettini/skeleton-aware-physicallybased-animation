using System.Collections;
using System.Collections.Generic;
using System.Text;
using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.SideChannels;
using BodyPart = Unity.MLAgentsExamples.BodyPart;



public class SkeletonSideChannel : SideChannel
{

    public SkeletonSideChannel()
    {
        ChannelId = new Guid("621f0a70-4f87-11ea-a6bf-784f4387d1f7");
    }

    protected override void OnMessageReceived(IncomingMessage msg)
    {
        // do nothing for now
    }

    public void SendSkeletonInfo(List<List<float>> initRotations, List<List<float>> initPositions)
    {
        using (var msgOut = new OutgoingMessage())
        {
            List<float> concatenatedMessage = new List<float>();

            // Add rotations to message Queue
            for (int i = 0; i<initRotations.Count; i++)
            {
                concatenatedMessage.Add(initRotations[i][0]);
                concatenatedMessage.Add(initRotations[i][1]);
                concatenatedMessage.Add(initRotations[i][2]);
                concatenatedMessage.Add(initRotations[i][3]);
            }
            // Add positions to message Queue
            for (int i = 0; i < initPositions.Count; i++)
            {
                concatenatedMessage.Add(initPositions[i][0]);
                concatenatedMessage.Add(initPositions[i][1]);
                concatenatedMessage.Add(initPositions[i][2]);
            }
            msgOut.WriteFloatList(concatenatedMessage);


            QueueMessageToSend(msgOut);
        }
    }
}
