//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    [Serializable]
    public class OpticalFlowRadMsg : Message
    {
        public const string k_RosMessageName = "mavros_msgs/OpticalFlowRad";
        public override string RosMessageName => k_RosMessageName;

        //  OPTICAL_FLOW_RAD message data
        public Std.HeaderMsg header;
        public uint integration_time_us;
        public float integrated_x;
        public float integrated_y;
        public float integrated_xgyro;
        public float integrated_ygyro;
        public float integrated_zgyro;
        public short temperature;
        public byte quality;
        public uint time_delta_distance_us;
        public float distance;

        public OpticalFlowRadMsg()
        {
            this.header = new Std.HeaderMsg();
            this.integration_time_us = 0;
            this.integrated_x = 0.0f;
            this.integrated_y = 0.0f;
            this.integrated_xgyro = 0.0f;
            this.integrated_ygyro = 0.0f;
            this.integrated_zgyro = 0.0f;
            this.temperature = 0;
            this.quality = 0;
            this.time_delta_distance_us = 0;
            this.distance = 0.0f;
        }

        public OpticalFlowRadMsg(Std.HeaderMsg header, uint integration_time_us, float integrated_x, float integrated_y, float integrated_xgyro, float integrated_ygyro, float integrated_zgyro, short temperature, byte quality, uint time_delta_distance_us, float distance)
        {
            this.header = header;
            this.integration_time_us = integration_time_us;
            this.integrated_x = integrated_x;
            this.integrated_y = integrated_y;
            this.integrated_xgyro = integrated_xgyro;
            this.integrated_ygyro = integrated_ygyro;
            this.integrated_zgyro = integrated_zgyro;
            this.temperature = temperature;
            this.quality = quality;
            this.time_delta_distance_us = time_delta_distance_us;
            this.distance = distance;
        }

        public static OpticalFlowRadMsg Deserialize(MessageDeserializer deserializer) => new OpticalFlowRadMsg(deserializer);

        private OpticalFlowRadMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.integration_time_us);
            deserializer.Read(out this.integrated_x);
            deserializer.Read(out this.integrated_y);
            deserializer.Read(out this.integrated_xgyro);
            deserializer.Read(out this.integrated_ygyro);
            deserializer.Read(out this.integrated_zgyro);
            deserializer.Read(out this.temperature);
            deserializer.Read(out this.quality);
            deserializer.Read(out this.time_delta_distance_us);
            deserializer.Read(out this.distance);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.integration_time_us);
            serializer.Write(this.integrated_x);
            serializer.Write(this.integrated_y);
            serializer.Write(this.integrated_xgyro);
            serializer.Write(this.integrated_ygyro);
            serializer.Write(this.integrated_zgyro);
            serializer.Write(this.temperature);
            serializer.Write(this.quality);
            serializer.Write(this.time_delta_distance_us);
            serializer.Write(this.distance);
        }

        public override string ToString()
        {
            return "OpticalFlowRadMsg: " +
            "\nheader: " + header.ToString() +
            "\nintegration_time_us: " + integration_time_us.ToString() +
            "\nintegrated_x: " + integrated_x.ToString() +
            "\nintegrated_y: " + integrated_y.ToString() +
            "\nintegrated_xgyro: " + integrated_xgyro.ToString() +
            "\nintegrated_ygyro: " + integrated_ygyro.ToString() +
            "\nintegrated_zgyro: " + integrated_zgyro.ToString() +
            "\ntemperature: " + temperature.ToString() +
            "\nquality: " + quality.ToString() +
            "\ntime_delta_distance_us: " + time_delta_distance_us.ToString() +
            "\ndistance: " + distance.ToString();
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