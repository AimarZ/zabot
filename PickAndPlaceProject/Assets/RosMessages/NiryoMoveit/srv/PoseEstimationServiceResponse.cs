//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.NiryoMoveit
{
    [Serializable]
    public class PoseEstimationServiceResponse : Message
    {
        public const string k_RosMessageName = "niryo_moveit/PoseEstimationService";
        public override string RosMessageName => k_RosMessageName;

        public Geometry.PointMsg estimated_position;
        public float estimated_scaleY;
        public int estimated_class;

        public PoseEstimationServiceResponse()
        {
            this.estimated_position = new Geometry.PointMsg();
            this.estimated_scaleY = 0.0f;
            this.estimated_class = 0;
        }

        public PoseEstimationServiceResponse(Geometry.PointMsg estimated_position, float estimated_scaleY, int estimated_class)
        {
            this.estimated_position = estimated_position;
            this.estimated_scaleY = estimated_scaleY;
            this.estimated_class = estimated_class;
        }

        public static PoseEstimationServiceResponse Deserialize(MessageDeserializer deserializer) => new PoseEstimationServiceResponse(deserializer);

        private PoseEstimationServiceResponse(MessageDeserializer deserializer)
        {
            this.estimated_position = Geometry.PointMsg.Deserialize(deserializer);
            deserializer.Read(out this.estimated_scaleY);
            deserializer.Read(out this.estimated_class);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.estimated_position);
            serializer.Write(this.estimated_scaleY);
            serializer.Write(this.estimated_class);
        }

        public override string ToString()
        {
            return "PoseEstimationServiceResponse: " +
            "\nestimated_position: " + estimated_position.ToString() +
            "\nestimated_scaleY: " + estimated_scaleY.ToString() +
            "\nestimated_class: " + estimated_class.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
