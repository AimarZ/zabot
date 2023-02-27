using System;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class SourceDestinationPublisher : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    public static readonly string[] LinkNames =
        { "world/base_link/shoulder_link", "/arm_link", "/elbow_link", "/forearm_link", "/wrist_link", "/hand_link" };

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/niryo_joints";

    [SerializeField]
    GameObject m_NiryoOne;
    // [SerializeField]
    // GameObject m_Target;
    // [SerializeField]
    // GameObject m_TargetPlacement;
    [SerializeField]
    GameObject m_RedCube;
    [SerializeField]
    GameObject m_GreenCube;
    [SerializeField]
    GameObject m_BlueCube;
    [SerializeField]
    GameObject m_RedCan;
    [SerializeField]
    GameObject m_GreenCan;
    [SerializeField]
    GameObject m_BlueCan;

    private GameObject[] pickObjects;
    private GameObject[] placeObjects;
    string textFile;
    List<int> orderList;
    string[] lines;

    static int next = 0;
    static int[] pickOrder;

    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    public SourceDestinationPublisher(){
        pickObjects = new GameObject[] {m_RedCube, m_GreenCube, m_BlueCube};
        placeObjects = new GameObject[]  {m_RedCan, m_GreenCan, m_BlueCan};
        //textFile = System.IO.Path.GetDirectoryName(Application.ExecutablePath) + "pickObjects.txt";
        
        // orderList = new List<int>();
        // lines = File.ReadAllLines(textFile);

        // foreach (string line in lines){
        //     int value = Int32.Parse(line);
        //     orderList.Add(value);
        // }
        //pickOrder = orderList.ToArray();
        pickOrder = new int[] {1,2,3};
    }

    void Start()
    {   

        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<NiryoMoveitJointsMsg>(m_TopicName);

        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_NiryoOne.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
        }
    }

    public void Publish()
    {
        var sourceDestinationMessage = new NiryoMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();
        }

        if (pickOrder.Length <= next){
            return;
        }

        // Pick Pose
        sourceDestinationMessage.pick_pose = new PoseMsg
        {
            position = pickObjects[pickOrder[next]].transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, pickObjects[pickOrder[next]].transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        sourceDestinationMessage.place_pose = new PoseMsg
        {
            position = placeObjects[pickOrder[next]].transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(m_TopicName, sourceDestinationMessage);
    }
}
