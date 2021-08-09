using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
using Random = UnityEngine.Random;

public class LaFanWalker : Agent
{
    [Header("Walk Speed")]
    [Range(0.1f, 10)]
    [SerializeField]
    //The walking speed to try and achieve
    private float m_TargetWalkingSpeed = 10;

    public float MTargetWalkingSpeed // property
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }

    const float m_maxWalkingSpeed = 10; //The max walking speed

    //Should the agent sample a new goal velocity each episode?
    //If true, walkSpeed will be randomly set between zero and m_maxWalkingSpeed in OnEpisodeBegin()
    //If false, the goal velocity will be walkingSpeed
    public bool randomizeWalkSpeedEachEpisode;
    public float scaleCompensation = 0.01f;

    //The direction an agent will walk during training.
    private Vector3 m_WorldDirToWalk = Vector3.right;

    [Header("Target To Walk Towards")] public Transform target; //Target the agent will walk towards during training.

    [Header("Body Parts")] 
    public Transform hips;
    public Transform spine;
    public Transform spine1;
    public Transform spine2;
    public Transform neck;
    public Transform head;
    public Transform thighL;
    public Transform shinL;
    public Transform footL;
    public Transform toeL;
    public Transform thighR;
    public Transform shinR;
    public Transform footR;
    public Transform toeR;
    public Transform shoulderL;
    public Transform armL;
    public Transform forearmL;
    public Transform handL;
    public Transform shoulderR;
    public Transform armR;
    public Transform forearmR;
    public Transform handR;


    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    //DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;
    EnvironmentParameters m_ResetParams;

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        //m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

        //Setup each body part
        m_JdController = GetComponent<JointDriveController>();
        m_JdController.SetupBodyPart(hips);
        m_JdController.SetupBodyPart(spine);
        m_JdController.SetupBodyPart(spine1);
        m_JdController.SetupBodyPart(spine2);
        m_JdController.SetupBodyPart(neck);
        m_JdController.SetupBodyPart(head);

        m_JdController.SetupBodyPart(thighL);
        m_JdController.SetupBodyPart(shinL);
        m_JdController.SetupBodyPart(footL);
        m_JdController.SetupBodyPart(toeL);

        m_JdController.SetupBodyPart(thighR);
        m_JdController.SetupBodyPart(shinR);
        m_JdController.SetupBodyPart(footR);
        m_JdController.SetupBodyPart(toeR);

        m_JdController.SetupBodyPart(shoulderL);
        m_JdController.SetupBodyPart(armL);
        m_JdController.SetupBodyPart(forearmL);
        m_JdController.SetupBodyPart(handL);

        m_JdController.SetupBodyPart(shoulderR);
        m_JdController.SetupBodyPart(armR);
        m_JdController.SetupBodyPart(forearmR);
        m_JdController.SetupBodyPart(handR);

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        SetResetParameters();
    }

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        //Reset all of the body parts
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        //Random start rotation to help generalize
        hips.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), -90);

        UpdateOrientationObjects();

        //Set our goal walking speed
        MTargetWalkingSpeed =
            randomizeWalkSpeedEachEpisode ? Random.Range(0.1f, m_maxWalkingSpeed) : MTargetWalkingSpeed;

        SetResetParameters();
    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        // Remove the ground check for now and check at the fixed update for early termination
        // sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

        //Get velocities in the context of our orientation cube's space
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.velocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));

        //Get position relative to hips in the context of our orientation cube's space
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.position));
        sensor.AddObservation(bp.rb.transform.localRotation);
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = m_OrientationCube.transform.forward;

        //velocity we want to match
        //var velGoal = cubeForward * MTargetWalkingSpeed;
        //ragdoll's avg vel
        var avgVel = GetAvgVelocity();

        //current ragdoll local velocity. 
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        //avg body vel relative to cube
        //sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        //vel goal relative to cube
        //sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));

        //rotation deltas
        //sensor.AddObservation(Quaternion.FromToRotation(hips.forward, cubeForward));
        //sensor.AddObservation(Quaternion.FromToRotation(head.forward, cubeForward));

        //Position of target position relative to orientation cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(target.transform.position));

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)

    {
        var bpDict = m_JdController.bodyPartsDict;
        var i = -1;

        var continuousActions = actionBuffers.ContinuousActions;

        bpDict[spine].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[spine1].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[spine2].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[neck].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[head].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        bpDict[thighL].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[shinL].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[footL].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[toeL].SetJointTargetRotation(continuousActions[++i], 0, 0);

        bpDict[thighR].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[shinR].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[footR].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[toeR].SetJointTargetRotation(continuousActions[++i], 0, 0);

        //bpDict[shoulderL].SetJointTargetRotation(0, 0, 0);
        bpDict[armL].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[forearmL].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[handL].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        //bpDict[shoulderL].SetJointTargetRotation(0, 0, 0);
        bpDict[armR].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[forearmR].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[handR].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        //update joint strength settings
        //bpDict[chest].SetJointStrength(continuousActions[++i]);
        bpDict[spine].SetJointStrength(continuousActions[++i]);
        bpDict[spine1].SetJointStrength(continuousActions[++i]);
        bpDict[spine2].SetJointStrength(continuousActions[++i]);
        bpDict[neck].SetJointStrength(continuousActions[++i]);
        bpDict[head].SetJointStrength(continuousActions[++i]);

        bpDict[thighL].SetJointStrength(continuousActions[++i]);
        bpDict[shinL].SetJointStrength(continuousActions[++i]);
        bpDict[footL].SetJointStrength(continuousActions[++i]);
        bpDict[toeL].SetJointStrength(continuousActions[++i]);

        bpDict[thighR].SetJointStrength(continuousActions[++i]);
        bpDict[shinR].SetJointStrength(continuousActions[++i]);
        bpDict[footR].SetJointStrength(continuousActions[++i]);
        bpDict[toeR].SetJointStrength(continuousActions[++i]);

        bpDict[armL].SetJointStrength(continuousActions[++i]);
        bpDict[forearmL].SetJointStrength(continuousActions[++i]);
        bpDict[handL].SetJointStrength(continuousActions[++i]);

        bpDict[armR].SetJointStrength(continuousActions[++i]);
        bpDict[forearmR].SetJointStrength(continuousActions[++i]);
        bpDict[handR].SetJointStrength(continuousActions[++i]);
        //Debug.Log("Number of acions : " + i.ToString());
    }

    //Update OrientationCube and DirectionIndicator
    void UpdateOrientationObjects()
    {
        m_WorldDirToWalk = target.position - hips.position;
        m_OrientationCube.UpdateOrientation(hips, target);
        //if (m_DirectionIndicator)
        //{
        //    m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        //}
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        var cubeForward = m_OrientationCube.transform.forward;

        //// Set reward for this step according to mixture of the following elements.
        //// a. Match target speed
        ////This reward will approach 1 if it matches perfectly and approach zero as it deviates
        //var matchSpeedReward = GetMatchingVelocityReward(cubeForward * MTargetWalkingSpeed, GetAvgVelocity());


        //// b. Rotation alignment with target direction.
        ////This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
        //var lookAtTargetReward = (Vector3.Dot(cubeForward, head.forward) + 1) * .5F;

        //AddReward(matchSpeedReward * lookAtTargetReward);

        float matchPositionReward = (float)Math.Pow(Vector3.Magnitude(target.position - m_OrientationCube.transform.position), 2f);
        matchPositionReward =(float)( 0.7f * Math.Exp(-0.5f* scaleCompensation * matchPositionReward));
        //Debug.Log("Matching Positiong Reward : " + matchPositionReward.ToString());


        //float matchSpeedReward = 1 - Vector3.Dot(GetAvgVelocity(), (target.position - m_OrientationCube.transform.position).normalized );
        Vector3 horizontalHipsSpeed = hips.GetComponent<Rigidbody>().velocity;
        horizontalHipsSpeed = new Vector3(horizontalHipsSpeed.x, 0f, horizontalHipsSpeed.z);

        Vector3 direction = (target.position - m_OrientationCube.transform.position).normalized;
        direction = new Vector3(direction.x, 0f, direction.z);

        float matchSpeedReward = 1 - Vector3.Dot(horizontalHipsSpeed, direction);
        matchSpeedReward = (float)(0.3f * Math.Exp( -Math.Pow( Math.Max(0f, matchSpeedReward) ,2f) ));
        //Debug.Log("1-dot : " + (1f - Vector3.Dot(horizontalHipsSpeed, direction)) + " , dot : " + Vector3.Dot(horizontalHipsSpeed, direction) + " , speed :" + matchSpeedReward);

        //Check for NaNs
        if (float.IsNaN(matchPositionReward))
        {
            throw new ArgumentException(
                "NaN in matchPositionReward.\n" +
                $" target position: {target.position}\n" +
                $" orientation cube position: {m_OrientationCube.transform.position}"
            );
        }

        //Check for NaNs
        if (float.IsNaN(matchSpeedReward))
        {
            throw new ArgumentException(
                "NaN in matchSpeedReward.\n" +
                $" forward: {(target.position - m_OrientationCube.transform.position).normalized }\n" +
                $" Average Velocity: {GetAvgVelocity()}"
            );
        }

        //Debug.Log("Position Reward : " + matchPositionReward.ToString());
        //Debug.Log("Distance to target : " + Vector3.Magnitude(target.position - m_OrientationCube.transform.position).ToString());

        //Debug.Log("Spped Reward" + matchSpeedReward.ToString());
        //Debug.Log("Velocity towards target : " + dotedDirection.ToString());



        float velMag = 0f;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velMag += item.rb.velocity.magnitude;
        }


        float speedConsReward = (float)(1f - Mathf.Exp(-0.00001f * Mathf.Pow(velMag, 2)));
        
        AddReward(matchPositionReward + matchSpeedReward);
        AddReward(-speedConsReward);

        //Debug.Log("Velocity Cons Reward : " + (-speedConsReward).ToString());
        Debug.Log("Full Reward : " + (matchSpeedReward + matchPositionReward - speedConsReward).ToString());

        //if (Vector3.Distance(target.position,m_OrientationCube.transform.position) < 2f || target.GetComponent<TargetController>().isTouched)
        //{
        //    Debug.Log("Touched Target");
        //    TouchedTarget();
        //    target.GetComponent<TargetController>().isTouched = false;
        //}

        if(hips.position.y < 0.2)
        {
            EndEpisode();
        }
    }

    //Returns the average velocity of all of the body parts
    //Using the velocity of the hips only has shown to result in more erratic movement from the limbs, so...
    //...using the average helps prevent this erratic movement
    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.velocity;
        }

        var avgVel = velSum / numOfRb;
        return avgVel;
    }

    //normalized value of the difference in avg speed vs goal walking speed.
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, MTargetWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / MTargetWalkingSpeed, 2), 2);
    }

    /// <summary>
    /// Agent touched the target
    /// </summary>
    public void TouchedTarget()
    {
        AddReward(1f);
    }

    public void SetTorsoMass()
    {
        m_JdController.bodyPartsDict[spine2].rb.mass = m_ResetParams.GetWithDefault("spine2_mass", 8);
        m_JdController.bodyPartsDict[spine1].rb.mass = m_ResetParams.GetWithDefault("spine1_mass", 8);
        m_JdController.bodyPartsDict[spine].rb.mass = m_ResetParams.GetWithDefault("spine_mass", 8);
        m_JdController.bodyPartsDict[hips].rb.mass = m_ResetParams.GetWithDefault("hip_mass", 10);
    }

    public void SetResetParameters()
    {
        SetTorsoMass();
    }
}
