using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RewardUI : MonoBehaviour
{

    public Slider upright;
    public Slider speed;
    public LaFanLegs agent;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        speed.value = agent.speedReward;
        upright.value = agent.uprightReward;
    }
}
