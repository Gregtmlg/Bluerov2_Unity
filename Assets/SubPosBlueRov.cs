using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using System.Collections;
using RosPose = RosMessageTypes.Geometry.PoseStampedMsg;


public class SubPosBlueRov : MonoBehaviour
{
    public GameObject obj;
    ROSConnection ros;
    public Vector3 offset = new Vector3(0,0.7f,0);

    public float RosPoseHz = 60.0f;
    private float PoseTimer = 0;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<RosPose>("bluerov/mavros/local_position/pose", SetUnityPose);
    }

    void SetUnityPose(RosPose bluerovPose)
    {
        PoseTimer += Time.deltaTime;

        if (PoseTimer > 1/RosPoseHz)
            PoseTimer = 0;
        else
            return;


        UnityEngine.Vector3 pos; 
        pos.x = (float)bluerovPose.pose.position.y; 
        pos.y = (float)bluerovPose.pose.position.z;
        pos.z = (float)bluerovPose.pose.position.x;

        Quaternion enuRotation = new Quaternion();
        Quaternion unityEulerAngles = new Quaternion();

        enuRotation.x = (float)bluerovPose.pose.orientation.x;
        enuRotation.y = (float)bluerovPose.pose.orientation.y;
        enuRotation.z = (float)bluerovPose.pose.orientation.z;
        enuRotation.w = (float)bluerovPose.pose.orientation.w;
        
        // Realised that ENU is in global frame, not the drone
        unityEulerAngles.eulerAngles = new Vector3(enuRotation.eulerAngles.y, 
            enuRotation.eulerAngles.z, 
            -enuRotation.eulerAngles.x);        
        // Debug.Log("unityEulerAngles: " + uav.transform.localRotation);

        // local_pos = pos;
        // local_rot = enuRotation.eulerAngles;

        obj.transform.position = pos + offset;
        obj.transform.rotation = unityEulerAngles;  
        obj.transform.Find("BlueROV2 Heavy__base_link").GetComponent<ArticulationBody>().TeleportRoot(obj.transform.position, obj.transform.rotation);
    }
}
