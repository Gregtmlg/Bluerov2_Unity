//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    [Serializable]
    public class MotorSetpointMsg : Message
    {
        public const string k_RosMessageName = "mavros_msgs/MotorSetpoint";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        //  motor throttle value [-1..1]
        public float[] setpoint;

        public MotorSetpointMsg()
        {
            this.header = new Std.HeaderMsg();
            this.setpoint = new float[8];
        }

        public MotorSetpointMsg(Std.HeaderMsg header, float[] setpoint)
        {
            this.header = header;
            this.setpoint = setpoint;
        }

        public static MotorSetpointMsg Deserialize(MessageDeserializer deserializer) => new MotorSetpointMsg(deserializer);

        private MotorSetpointMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.setpoint, sizeof(float), 8);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.setpoint);
        }

        public override string ToString()
        {
            return "MotorSetpointMsg: " +
            "\nheader: " + header.ToString() +
            "\nsetpoint: " + System.String.Join(", ", setpoint.ToList());
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