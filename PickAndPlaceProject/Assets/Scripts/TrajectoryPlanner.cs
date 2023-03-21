using System;
using System.IO;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "niryo_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_NiryoOne;
    public GameObject NiryoOne { get => m_NiryoOne; set => m_NiryoOne = value; }

    [SerializeField]
    GameObject m_RedCan;
    public GameObject RedCan { get => m_RedCan; set => m_RedCan = value; }
    [SerializeField]
    GameObject m_GreenCan;
    public GameObject GreenCan { get => m_GreenCan; set => m_GreenCan = value; }
    [SerializeField]
    GameObject m_BlueCan;
    public GameObject BlueCan { get => m_BlueCan; set => m_BlueCan = value; }

    [SerializeField]
    GameObject m_RedCube;
    public GameObject RedCube { get => m_RedCube; set => m_RedCube = value; }
    [SerializeField]
    GameObject m_GreenCube;
    public GameObject GreenCube { get => m_GreenCube; set => m_GreenCube = value; }
    [SerializeField]
    GameObject m_BlueCube;
    public GameObject BlueCube { get => m_BlueCube; set => m_BlueCube = value; }

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


    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickOffset = Vector3.up * 0.1f;
    readonly Vector3 m_PlaceOffset = Vector3.up * 0.3f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

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
        byte[] imageBytes = rgbd_im.EncodeToPNG();
        //rgbd_im.Apply();
        RenderTexture.active = null;
        //byte[] imageBytes = rgbd_im.GetRawTextureData();

        return imageBytes;
    }


    
    /// <summary>
    ///     Create a new PoseEstimationServiceRequest with the captured screenshot as bytes and instantiates 
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
        Debug.Log("10");
	}

    void PoseEstimationCallback(PoseEstimationServiceResponse response)
	{
        Debug.Log("11");
        /*if (response != null)
        {
            // The position output by the model is the position of the cube relative to the camera so we need to extract its global position 
            var estimatedPosition = Camera.main.transform.TransformPoint(response.estimated_pose.position.From<RUF>());
            var estimatedRotation = Camera.main.transform.rotation * response.estimated_pose.orientation.From<RUF>();

            PublishJoints(estimatedPosition, estimatedRotation);

            EstimatedPos.text = estimatedPosition.ToString();
            EstimatedRot.text = estimatedRotation.eulerAngles.ToString();
        }
        else {
            InitializeButton.interactable = true;
            RandomizeButton.interactable = true;
        }
        */
        Debug.Log("HEllo");
    }

        /// <summary>
    ///     Button callback for the Pose Estimation
    /// </summary>
    public void PoseEstimation(){
        Debug.Log("Capturing screenshot...");

        // InitializeButton.interactable = false;
        // RandomizeButton.interactable = false;
        // ServiceButton.interactable = false;
        // ActualPos.text = target.transform.position.ToString();
        // ActualRot.text = target.transform.eulerAngles.ToString();
        // EstimatedPos.text = "-";
        // EstimatedRot.text = "-";

        // Capture the screenshot and pass it to the pose estimation service
        byte[] pngBytes = CaptureScreenshot();
        // uint imageHeight = (uint)renderTexture.height;
        // uint imageWidth = (uint)renderTexture.width;
        // Texture2D target = new Texture2D((int)imageWidth,(int)imageHeight);
        // target.LoadRawTextureData(rawImageData);
        // target.Apply();
        // byte[] pngBytes = target.EncodeToPNG();
        File.WriteAllBytes(dir+"screen.png", pngBytes);
        InvokePoseEstimationService(pngBytes);
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

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Find left and right fingers
        var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

        m_RightGripper = m_NiryoOne.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_NiryoOne.transform.Find(leftGripper).GetComponent<ArticulationBody>();

        // Render texture 
        renderTexture = new RenderTexture(Camera.main.pixelWidth, Camera.main.pixelHeight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        renderTexture.Create();
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
    public void PublishJoints()
    {
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        pickObjects = new GameObject[] {m_RedCube, m_GreenCube, m_BlueCube};
        placeObjects = new GameObject[]  {m_RedCan, m_GreenCan, m_BlueCan};

        lines = File.ReadAllLines(dir+textFile);
        ind = Int32.Parse(lines[0]);
        File.WriteAllLines(dir+textFile, lines.Skip(1).ToArray());

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (pickObjects[ind].transform.position + m_PickOffset).To<FLU>(),

            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(90, pickObjects[ind].transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (placeObjects[ind].transform.position + m_PlaceOffset).To<FLU>(),
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

            // All trajectories have been executed, open the gripper to place the target cube
            //OpenGripper();
        }
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
