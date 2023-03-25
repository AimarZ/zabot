using System;
using System.IO;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class GetFrameRGBD : MonoBehaviour
{
    static int frame = -3;
    string dir = "Assets/Captures~/";

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
    ///     Button callback for the Pose Estimation
    /// </summary>
    public void Update(){
        //Debug.Log("Capturing screenshot...");

        // InitializeButton.interactable = false;
        // RandomizeButton.interactable = false;
        // ServiceButton.interactable = false;
        // ActualPos.text = target.transform.position.ToString();
        // ActualRot.text = target.transform.eulerAngles.ToString();
        // EstimatedPos.text = "-";
        // EstimatedRot.text = "-";
        frame++;
        if (frame>-1){
            // Capture the screenshot and pass it to the pose estimation service
            byte[] pngBytes = CaptureScreenshot();
            // uint imageHeight = (uint)renderTexture.height;
            // uint imageWidth = (uint)renderTexture.width;
            // Texture2D target = new Texture2D((int)imageWidth,(int)imageHeight);
            // target.LoadRawTextureData(rawImageData);
            // target.Apply();
            // byte[] pngBytes = target.EncodeToPNG();
            
            File.WriteAllBytes(dir+"screen"+frame.ToString()+".png", pngBytes);
        }
        
    }

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        return;
    }

}
