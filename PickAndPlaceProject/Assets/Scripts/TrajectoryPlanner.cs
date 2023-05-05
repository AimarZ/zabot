using System;
using System.IO;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using RosMessageTypes.Moveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.UI;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables requiYellow for ROS communication
    [SerializeField]
    string m_RosServiceName = "niryo_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_NiryoOne;
    public GameObject NiryoOne { get => m_NiryoOne; set => m_NiryoOne = value; }

    [SerializeField]
    GameObject m_GreenCan;
    public GameObject GreenCan { get => m_GreenCan; set => m_GreenCan = value; }
    [SerializeField]
    GameObject m_BlueCan;
    public GameObject BlueCan { get => m_BlueCan; set => m_BlueCan = value; }       
    [SerializeField]
    GameObject m_YellowCan;
    public GameObject YellowCan { get => m_YellowCan; set => m_YellowCan = value; }

    [SerializeField]
    GameObject m_GreenCube1;
    public GameObject GreenCube1 { get => m_GreenCube1; set => m_GreenCube1 = value; }
    [SerializeField]
    GameObject m_GreenCube2;
    public GameObject GreenCube2 { get => m_GreenCube2; set => m_GreenCube2 = value; }
    [SerializeField]
    GameObject m_BlueCube1;
    public GameObject BlueCube1 { get => m_BlueCube1; set => m_BlueCube1 = value; }
    [SerializeField]
    GameObject m_BlueCube2;
    public GameObject BlueCube2 { get => m_BlueCube2; set => m_BlueCube2 = value; }
    [SerializeField]
    GameObject m_YellowCube1;
    public GameObject YellowCube1 { get => m_YellowCube1; set => m_YellowCube1 = value; }
    [SerializeField]
    GameObject m_YellowCube2;
    public GameObject YellowCube2 { get => m_YellowCube2; set => m_YellowCube2 = value; }

    private GameObject[] pickObjects;
    private GameObject[] placeObjects;
    string textFile = "pickObjects.txt";
    string dir = "Assets/Custom/";
    string[] lines;
    int ind = -1;

    //From Pose Estimation
    private RenderTexture renderTexture;
    private const int isBigEndian = 0;
    private const int step = 4;

    private Button RandomizeButton;
    private Button InitializeButton;
    private Button PublishButton;
    private Button RandomizeButton2;
    private Button PublishButton2;
    private Button InitializeButton2;

    private GameObject niryo_shoulder_link;

    public PoseEstimationScenario scenario;

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    //readonly Vector3 m_PickOffset = Vector3.up * 0.075f;
    readonly Vector3 m_PickOffset = Vector3.up * 0.17f;
    readonly Vector3 m_PlaceOffset = Vector3.up * 0.3f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Button callback for the Cube Randomization
    /// </summary>
    public void RandomizeCube(){
        scenario.Move();
        // ActualPos.text = target.transform.position.ToString();
        // ActualRot.text = target.transform.eulerAngles.ToString();
    }

    /// <summary>
    ///     Button callback for setting the robot to default position
    /// </summary>
    public void Initialize(){
        PublishButton.interactable = false;
        PublishButton2.interactable = false;
        InitializeButton.interactable = false;
        InitializeButton2.interactable = false;
        StartCoroutine(MoveToInitialPosition());
    }

    private IEnumerator MoveToInitialPosition()
    {
        bool isRotationFinished = false;
        while (!isRotationFinished)
        {
            isRotationFinished = ResetRobotToDefaultPosition2();
            yield return new WaitForSeconds(k_JointAssignmentWait);
        }
        InitializeButton.interactable = true;
        InitializeButton2.interactable = true;
        PublishButton.interactable = true;
        PublishButton2.interactable = true;
    }

    private bool ResetRobotToDefaultPosition()
    {
        bool isRotationFinished = true;
        var rotationSpeed = 90f;

        for (int i = 0; i < k_NumRobotJoints; i++)
        {
            var tempXDrive = m_JointArticulationBodies[i].xDrive;
            float currentRotation = tempXDrive.target;
            
            float rotationChange = rotationSpeed * Time.fixedDeltaTime;
            
            if (currentRotation > 0f) rotationChange *= -1;
            
            if (Mathf.Abs(currentRotation) < rotationChange)
                rotationChange = 0;
            else
                isRotationFinished = false;
            
            // the new xDrive target is the currentRotation summed with the desired change
            float rotationGoal = currentRotation + rotationChange;
            tempXDrive.target = rotationGoal;
            m_JointArticulationBodies[i].xDrive = tempXDrive;
        }
        return isRotationFinished;
    }

    private bool ResetRobotToDefaultPosition2()
    {
        bool isRotationFinished = true;
        var rotationSpeed = 120f;

        for (int i = 0; i < k_NumRobotJoints; i++)
        {
            var tempXDrive = m_JointArticulationBodies[i].xDrive;
            float currentRotation = tempXDrive.target;
            float rotationChange = rotationSpeed * Time.fixedDeltaTime;
            float targetRotation = 0f;
            
            if (i==0)
                targetRotation = 130f;

            if (currentRotation > targetRotation) rotationChange *= -1;
            
            if (Mathf.Abs(currentRotation-targetRotation) < rotationChange)
                rotationChange = 0;
            else
                isRotationFinished = false;
            
            // the new xDrive target is the currentRotation summed with the desired change
            float rotationGoal = currentRotation + rotationChange;
            tempXDrive.target = rotationGoal;
            m_JointArticulationBodies[i].xDrive = tempXDrive;
        }
        return isRotationFinished;
    }


    /// <summary>
    ///     Capture the main camera's render texture and convert to bytes.
    /// </summary>
    /// <returns>imageBytes</returns>
    private byte[] CaptureScreenshot()
    {
        // Camera.main.depthTextureMode = DepthTextureMode.Depth;
        // Camera.main.targetTexture = renderTexture;
        // RenderTexture currentRT = RenderTexture.active;
        // RenderTexture.active = renderTexture;
        // Camera.main.Render();
        // Texture2D mainCameraTexture = new Texture2D(renderTexture.width, renderTexture.height);
        // mainCameraTexture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        // mainCameraTexture.Apply();
        // RenderTexture.active = currentRT;
        // // Get the raw byte info from the screenshot
        // byte[] imageBytes = mainCameraTexture.GetRawTextureData();
        // Camera.main.targetTexture = null;
        // return imageBytes;

        Camera.main.depthTextureMode = DepthTextureMode.Depth;
        Camera cam = Camera.main;
        RenderTexture render_tex = cam.targetTexture;
        int W = render_tex.width;
        int H = render_tex.height;
        //far_clip = cam.farClipPlane;

        // we will get the RGB-D image in this Texture2D on the CPU
        Texture2D rgbd_im = new Texture2D(W, H, TextureFormat.RGBAFloat, false);

        // GPU -> CPU
        RenderTexture prev = RenderTexture.active;
        RenderTexture.active = render_tex;
        rgbd_im.ReadPixels(new Rect(0, 0, W, H), 0, 0);
        byte[] pngBytes = rgbd_im.EncodeToPNG();
        //rgbd_im.Apply();
        RenderTexture.active = null;
        byte[] imageBytes = rgbd_im.GetRawTextureData();
        imageBytes = pngBytes;

        return imageBytes;
    }


    
    /// <summary>
    ///     Create a new PoseEstimationServiceRequest with the captuYellow screenshot as bytes and instantiates 
    ///     a sensor_msgs/image.
    ///
    ///     Call the PoseEstimationService using the ROSConnection and calls PoseEstimationCallback on the 
    ///     PoseEstimationServiceResponse.
    /// </summary>
    /// <param name="imageData"></param>
    private void InvokePoseEstimationService(byte[] imageData)
    {
        uint imageHeight = (uint)renderTexture.height;
        uint imageWidth = (uint)renderTexture.width;
        RosMessageTypes.Sensor.ImageMsg rosImage = new RosMessageTypes.Sensor.ImageMsg(new RosMessageTypes.Std.HeaderMsg(), imageWidth, imageHeight, "RGBA", isBigEndian, step, imageData);
        PoseEstimationServiceRequest poseServiceRequest = new PoseEstimationServiceRequest(rosImage);           
        m_Ros.SendServiceMessage<PoseEstimationServiceResponse>("pose_estimation_srv", poseServiceRequest, PoseEstimationCallback);

	}

    void PoseEstimationCallback(PoseEstimationServiceResponse response)
	{
        if (response != null)
        {
            
            // The position output by the model is the position of the cube relative to the camera so we need to extract its global position 
            var estimated_position = Camera.main.transform.TransformPoint(response.estimated_position.From<RUF>());
            var estimated_scaleY = response.estimated_scaleY;
            var estimated_class = response.estimated_class; 

            PublishJoints(estimated_position, estimated_scaleY,estimated_class);

        }
        else {         
            RandomizeButton.interactable = true;
            InitializeButton.interactable = true;
            PublishButton.interactable = true;
            RandomizeButton2.interactable = true;
            PublishButton2.interactable = true;
            InitializeButton2.interactable = true;
        }
    }



    /// <summary>
    ///     Button callback for the Pose Estimation
    /// </summary>
    public void PoseEstimation(){
        Debug.Log("Capturing screenshot...");

        RandomizeButton.interactable = false;
        InitializeButton.interactable = false;
        PublishButton.interactable = false;
        RandomizeButton2.interactable = false;
        PublishButton2.interactable = false;
        InitializeButton2.interactable = false;
        // ServiceButton.interactable = false;
        // ActualPos.text = target.transform.position.ToString();
        // ActualRot.text = target.transform.eulerAngles.ToString();
        // EstimatedPos.text = "-";
        // EstimatedRot.text = "-";

        // Capture the screenshot and pass it to the pose estimation service

        byte[] imgBytes = CaptureScreenshot();

        // niryo_shoulder_link.transform.eulerAngles = new Vector3(
        //     niryo_shoulder_link.transform.eulerAngles.x,
        //     0,
        //     niryo_shoulder_link.transform.eulerAngles.z
        // );
        // uint imageHeight = (uint)renderTexture.height;
        // uint imageWidth = (uint)renderTexture.width;
        // Texture2D target = new Texture2D((int)imageWidth,(int)imageHeight);
        // target.LoadRawTextuYellowata(rawImageData);
        // target.Apply();
        // byte[] pngBytes = target.EncodeToPNG();
        
        //File.WriteAllBytes(dir+"screen.png", pngBytes);
        InvokePoseEstimationService(imgBytes);
    }

    

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>NiryoMoveitJoints</returns>
    NiryoMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new NiryoMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints(Vector3 targetPos, float targetScaleY, int label)
    {   
        if (label == 0){
            Debug.Log("There is no object");
            return;
        }

        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();;

        int objind = 0;
        float maxHeight = 0;
        for(int i=0;i<=5;i++){
            if (pickObjects[i].transform.position.y > 0 && pickObjects[i].transform.position.y < 10 && pickObjects[i].GetComponent<MeshRenderer>().bounds.size.y > maxHeight){
                objind = i;
                maxHeight = pickObjects[i].GetComponent<MeshRenderer>().bounds.size.y;
            }
        }
        //Vector3 scaleWorld = transform.localToWorldMatrix.MultiplyVector(pickObjects[ind].GetComponent<BoxCollider>().size);
        Vector3 size = pickObjects[objind].GetComponent<MeshRenderer>().bounds.size; 
        Debug.Log("Sizes:");
        Debug.Log(size*100);
        Debug.Log(targetScaleY*100);
        //targetPos = targetPos/100;
        
        request.scaleY = size.y;
        
        Debug.Log("Positions:");
        Debug.Log(pickObjects[objind].transform.position*100);
        Debug.Log(targetPos*100);


        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            //position = (targetPos + m_PickOffset+ Vector3.up*targetScaleY).To<FLU>(),
            position = (targetPos + m_PickOffset).To<FLU>(),
            

            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(90, pickObjects[objind].transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (placeObjects[label-1].transform.position + m_PlaceOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
            RandomizeButton.interactable = true;
            PublishButton.interactable = true;
            InitializeButton.interactable = true;
            RandomizeButton2.interactable = true;
            PublishButton2.interactable = true;
            InitializeButton2.interactable = true;
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from niryo_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }
                else if (poseIndex == (int)Poses.Place2)
                {
                    OpenGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            PublishButton.interactable = true;
            RandomizeButton.interactable = true;
            InitializeButton.interactable = true;
            PublishButton2.interactable = true;
            RandomizeButton2.interactable = true;
            InitializeButton2.interactable = true;

            // All trajectories have been executed, open the gripper to place the target cube
            //OpenGripper();
        }
    }

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);
        m_Ros.RegisterRosService<PoseEstimationServiceRequest, PoseEstimationServiceResponse>("pose_estimation_srv");

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        pickObjects = new GameObject[] {m_GreenCube1, m_GreenCube2,  m_BlueCube1, m_BlueCube2, m_YellowCube1, m_YellowCube2};
        placeObjects = new GameObject[]  { m_GreenCan, m_BlueCan, m_YellowCan };

        // Find left and right fingers
        var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

        m_RightGripper = m_NiryoOne.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_NiryoOne.transform.Find(leftGripper).GetComponent<ArticulationBody>();

        RandomizeButton = GameObject.Find("Display1Buttons/RandomizeButton").GetComponent<Button>();
        InitializeButton = GameObject.Find("Display1Buttons/InitializeButton").GetComponent<Button>();
        PublishButton = GameObject.Find("Display1Buttons/PublishButton").GetComponent<Button>();
        RandomizeButton2 = GameObject.Find("Display2Buttons/RandomizeButton").GetComponent<Button>();
        PublishButton2 = GameObject.Find("Display2Buttons/PublishButton").GetComponent<Button>();
        InitializeButton2 = GameObject.Find("Display2Buttons/InitializeButton").GetComponent<Button>();

        // Render texture 
        renderTexture = new RenderTexture(Camera.main.pixelWidth, Camera.main.pixelHeight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        renderTexture.Create();
    }

    enum Poses
    {
        PreGrasp,
        PreGrasp2,
        Grasp,
        PickUp,
        PickUp2,
        Place,
        Place2,
        PostPlace
    }
}
