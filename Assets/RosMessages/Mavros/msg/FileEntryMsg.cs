//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    [Serializable]
    public class FileEntryMsg : Message
    {
        public const string k_RosMessageName = "mavros_msgs/FileEntry";
        public override string RosMessageName => k_RosMessageName;

        //  File/Dir information
        public const byte TYPE_FILE = 0;
        public const byte TYPE_DIRECTORY = 1;
        public string name;
        public byte type;
        public ulong size;
        //  Not supported by MAVLink FTP
        // time atime
        // int32 access_flags

        public FileEntryMsg()
        {
            this.name = "";
            this.type = 0;
            this.size = 0;
        }

        public FileEntryMsg(string name, byte type, ulong size)
        {
            this.name = name;
            this.type = type;
            this.size = size;
        }

        public static FileEntryMsg Deserialize(MessageDeserializer deserializer) => new FileEntryMsg(deserializer);

        private FileEntryMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.name);
            deserializer.Read(out this.type);
            deserializer.Read(out this.size);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.name);
            serializer.Write(this.type);
            serializer.Write(this.size);
        }

        public override string ToString()
        {
            return "FileEntryMsg: " +
            "\nname: " + name.ToString() +
            "\ntype: " + type.ToString() +
            "\nsize: " + size.ToString();
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
