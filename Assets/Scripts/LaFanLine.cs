using System;
using System.Collections.Generic;

using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
using Random = UnityEngine.Random;
using Unity.MLAgents.SideChannels;

public class LaFanLine : Agent
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
    [HideInInspector] public JointDriveController m_JdController;
    EnvironmentParameters m_ResetParams;
    float[,] pose_rotations;

    public Transform RootRef;

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        //m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();


        if (target == null)
        {
            target = new GameObject().GetComponent<Transform>();
            target.SetParent(transform);
            target.name = "target";
            target.position = new Vector3(-100f, 0f, transform.position.z);
        }

        //Setup each body part
        m_JdController = GetComponent<JointDriveController>();
        m_JdController.root = RootRef;
        m_JdController.SetupBodyPart(hips);

        m_JdController.SetupBodyPart(thighL);
        m_JdController.SetupBodyPart(shinL);
        m_JdController.SetupBodyPart(footL);
        m_JdController.SetupBodyPart(toeL);

        m_JdController.SetupBodyPart(thighR);
        m_JdController.SetupBodyPart(shinR);
        m_JdController.SetupBodyPart(footR);
        m_JdController.SetupBodyPart(toeR);

        m_JdController.SetupBodyPart(spine);
        m_JdController.SetupBodyPart(spine1);
        m_JdController.SetupBodyPart(spine2);
        m_JdController.SetupBodyPart(neck);
        m_JdController.SetupBodyPart(head);

        m_JdController.SetupBodyPart(shoulderL);
        m_JdController.SetupBodyPart(armL);
        m_JdController.SetupBodyPart(forearmL);
        m_JdController.SetupBodyPart(handL);

        m_JdController.SetupBodyPart(shoulderR);
        m_JdController.SetupBodyPart(armR);
        m_JdController.SetupBodyPart(forearmR);
        m_JdController.SetupBodyPart(handR);

        pose_rotations = Load_Init_Pos.Read_CSV("Text/walk1_subject1_csv");
        Debug.Log(pose_rotations.GetLength(0));
        Debug.Log(pose_rotations.GetLength(1));
        m_ResetParams = Academy.Instance.EnvironmentParameters;

        SetResetParameters();
    }

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        int count = 0;
        //Reset all of the body parts
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            int pose = Random.Range(80, 900);//pose_rotations.GetLength(0));

            Quaternion temp_rot;
            temp_rot = bodyPart.startingRot;
            if (count == 0)
            {
            //    //Vector3 euler = new Vector3(0f, -178f, -70f);
            //    //temp_rot = Quaternion.Euler(euler);
                //temp_rot = bodyPart.startingRot;
                Quaternion poseRot = new Quaternion(pose_rotations[pose, count + 4], pose_rotations[pose, count + 5], pose_rotations[pose, count + 6], pose_rotations[pose, count + 3]);
                //Quaternion globrot = bodyPart.rb.transform.parent.rotation* poseRot;
                float z = (Quaternion.Inverse(bodyPart.startingLocalRot) * poseRot).eulerAngles.z;
                float x = (Quaternion.Inverse(bodyPart.startingLocalRot) * poseRot).eulerAngles.x;
                Quaternion deltaRot = Quaternion.Inverse(bodyPart.startingLocalRot) * poseRot;
                temp_rot = poseRot * Quaternion.AngleAxis(-deltaRot.eulerAngles.y, Vector3.up) ;

                
                //temp_rot = Quaternion.AngleAxis(z, Vector3.forward) * Quaternion.AngleAxis(x, Vector3.right) * bodyPart.startingLocalRot;
                //temp_rot = Quaternion.AngleAxis(z, bodyPart.rb.transform.forward) * Quaternion.AngleAxis(x, bodyPart.rb.transform.right) * bodyPart.startingLocalRot;
            }
            else
            {

            //    Vector3 euler = new Vector3(pose_rotations[pose, count + 2], pose_rotations[pose, count + 1], pose_rotations[pose, count + 0]);
            // load in order x,y,z,w
            temp_rot = new Quaternion(pose_rotations[pose, count+4], pose_rotations[pose, count + 5], pose_rotations[pose, count + 6], pose_rotations[pose, count + 3]);
            //    //temp_rot = Load_Init_Pos.ToQuaternion(euler);
            //    temp_rot = Load_Init_Pos.BvhToUnityRotation(euler, AxisOrder.ZYX);
            //    //temp_rot = new Quaternion(-temp_rot.z, -temp_rot.x, temp_rot.y, temp_rot.w);
            //    //temp_rot = Quaternion.Euler(0f, 0f, -90f) * temp_rot;
            //    //temp_rot = new Quaternion(pose_rotations[pose, count + 3], pose_rotations[pose, count + 1], pose_rotations[pose, count + 2], pose_rotations[pose, count + 0]);

            }

            bodyPart.Reset(bodyPart, temp_rot);

            count += 7;
        }

        //Random start rotation to help generalize
        //hips.rotation = Quaternion.Euler(0, 180, -90);
        hips.parent.rotation = Quaternion.Euler(-90, 0, -90);

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
        //sensor.AddObservation(hips.transform.InverseTransformDirection(bp.rb.velocity));
        //sensor.AddObservation(hips.transform.InverseTransformVector(bp.rb.velocity));
        sensor.AddObservation(RootRef.transform.InverseTransformVector(bp.rb.velocity));
        sensor.AddObservation(RootRef.transform.InverseTransformVector(bp.rb.angularVelocity));

        //Get position relative to hips in the context of our orientation cube's space
        //sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.transform.position));
        sensor.AddObservation(RootRef.transform.InverseTransformPoint(bp.rb.transform.position));

        //sensor.AddObservation(Quaternion.Inverse(m_OrientationCube.transform.rotation) * bp.rb.transform.rotation);
        //sensor.AddObservation(Quaternion.Inverse(hips.localRotation) * bp.rb.transform.localRotation);
        sensor.AddObservation(bp.rb.transform.localRotation);
        //Debug.Log(bp.rb.name + " : " + bp.rb.transform.localRotation);

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
        var avgVel = CenterOfMassVelocity();

        //avg center of mass body vel relative to cube
        //vel goal relative to cube
        //sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));

        //rotation deltas
        //sensor.AddObservation(Quaternion.FromToRotation(hips.forward, cubeForward));
        //sensor.AddObservation(Quaternion.FromToRotation(head.forward, cubeForward));


        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }

        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        //Position of target position relative to orientation cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(target.transform.position));
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
        m_WorldDirToWalk = Vector3.right; //target.position - hips.position;
        
        m_OrientationCube.UpdateOrientation(hips, target);
        //if (m_DirectionIndicator)
        //{
        //    m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        //}
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();
        //AverageVelocityPerJoint();

        ComputeReward();

        if (hips.position.y < 0.2)
        {
            EndEpisode();
        }
    }

    private void AverageVelocityPerJoint()
    {
        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            bodyPart.UpdateVelBuffer(RootRef.transform.InverseTransformVector(bodyPart.rb.velocity));
        }
    }

    private void ComputeReward()
    {
        Vector3 avgVel = CenterOfMassVelocity();
        

        Vector3 direction = (target.position - m_OrientationCube.transform.position).normalized;
        direction = new Vector3(direction.x, 0f, direction.z);

        //Debug.DrawRay(hips.position, avgVel, Color.blue);
        //Debug.DrawRay(hips.position, direction, Color.green);
        //Debug.DrawLine(hips.position, hips.position + direction * Vector3.Dot(avgVel, direction), Color.red);

        float matchSpeedReward = MTargetWalkingSpeed - Vector3.Dot(avgVel, direction);
        matchSpeedReward = Mathf.Exp(-0.25f*Mathf.Pow(Mathf.Max(0f, matchSpeedReward), 2f));
        //Debug.Log("Velocity : " + avgVel);
        //Debug.Log("Match Speed reward : " + matchSpeedReward);

        // Compute the overall velocity of each joint 
        float velMag = 0f;
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velMag += item.rb.velocity.magnitude;
        }
        // Penalize fast movement to enable energy conservation
        float speedConsReward = 1f - Mathf.Exp(-0.0001f * Mathf.Pow(velMag, 2));
        speedConsReward = 0f;

        AddReward(matchSpeedReward - speedConsReward);

        //Debug.Log("Full Reward : " + (matchSpeedReward - speedConsReward).ToString());
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

    // Center of Mass Velocity, takes in account the mass of the body
    // Body parts with less weight have less effect on the overal velocity
    Vector3 CenterOfMassVelocity()
    {
        Vector3 velSum = Vector3.zero;

        //ALL RBS
        float totalMass = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            velSum += item.rb.velocity * item.rb.mass;
            totalMass += item.rb.mass;
        }

        var avgVel = velSum / totalMass;
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
