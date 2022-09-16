//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Geometry
{
    [Serializable]
    public class TwistMsg : Message
    {
        public const string k_RosMessageName = "geometry_msgs/Twist";
        public override string RosMessageName => k_RosMessageName;

        //  This expresses velocity in free space broken into its linear and angular parts.
        public Vector3Msg linear;
        public Vector3Msg angular;

        public TwistMsg()
        {
            this.linear = new Vector3Msg();
            this.angular = new Vector3Msg();
        }

        public TwistMsg(Vector3Msg linear, Vector3Msg angular)
        {
            this.linear = linear;
            this.angular = angular;
        }

        public static TwistMsg Deserialize(MessageDeserializer deserializer) => new TwistMsg(deserializer);

        private TwistMsg(MessageDeserializer deserializer)
        {
            this.linear = Vector3Msg.Deserialize(deserializer);
            this.angular = Vector3Msg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.linear);
            serializer.Write(this.angular);
        }

        public override string ToString()
        {
            return "TwistMsg: " +
            "\nlinear: " + linear.ToString() +
            "\nangular: " + angular.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
