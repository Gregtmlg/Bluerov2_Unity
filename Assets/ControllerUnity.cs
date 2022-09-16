using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;
using RosVelMsg = RosMessageTypes.Mavros.MotorSetpointMsg;
// using RosVelMsg = RosMessageTypes.Geometry.TwistStampedMsg;

namespace RosSharp.Control
{

    public class ControllerUnity : MonoBehaviour
    {
        public GameObject thruster1;
        public GameObject thruster2;
        public GameObject thruster3;
        public GameObject thruster4;
        public GameObject thruster5;
        public GameObject thruster6;
        public GameObject thruster7;
        public GameObject thruster8;
        // public ControlMode mode = ControlMode.ROS;

        private ArticulationBody tA1;
        private ArticulationBody tA2;
        private ArticulationBody tA3;
        private ArticulationBody tA4;
        private ArticulationBody tA5;
        private ArticulationBody tA6;
        private ArticulationBody tA7;
        private ArticulationBody tA8;
        // private float StateTimer = 0; private float VelTimer = 0; 
        // public float RosVelHz = 30.0f; public float RosGlobalHz = 50.0f;

        public float maxLinearSpeed = 2; //  m/s
        public float maxRotationalSpeed = 1;//
        public float wheelRadius = 0.033f; //meters
        public float trackWidth = 0.288f; // meters Distance between tyres
        public float forceLimit = 10;
        public float damping = 10;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;

        ROSConnection ros;
        private RotationDirection direction;
        private float[] rosThrust = new float[8];


        void Start()
        {
            tA1 = thruster1.GetComponent<ArticulationBody>();
            tA2 = thruster2.GetComponent<ArticulationBody>();
            tA3 = thruster3.GetComponent<ArticulationBody>();
            tA4 = thruster4.GetComponent<ArticulationBody>();
            tA5 = thruster5.GetComponent<ArticulationBody>();
            tA6 = thruster6.GetComponent<ArticulationBody>();
            tA7 = thruster7.GetComponent<ArticulationBody>();
            tA8 = thruster8.GetComponent<ArticulationBody>();
            SetParameters(tA1);
            SetParameters(tA2);
            SetParameters(tA3);
            SetParameters(tA4);
            SetParameters(tA5);
            SetParameters(tA6);
            SetParameters(tA7);
            SetParameters(tA8);
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<RosVelMsg>("bluerov/mavros/setpoint_motor/setpoint", ReceiveROSCmd);
        }

        void ReceiveROSCmd(RosVelMsg cmdvel)
        {
        
            SetSpeed(tA1, cmdvel.setpoint[0]);
            SetSpeed(tA2, cmdvel.setpoint[1]);
            SetSpeed(tA3, cmdvel.setpoint[2]);
            SetSpeed(tA4, cmdvel.setpoint[3]);
            SetSpeed(tA5, cmdvel.setpoint[4]);
            SetSpeed(tA6, cmdvel.setpoint[5]);
            SetSpeed(tA7, cmdvel.setpoint[6]);
            SetSpeed(tA8, cmdvel.setpoint[7]);
        }

        // void FixedUpdate()
        // {
        //     if (mode == ControlMode.Keyboard)
        //     {
        //         KeyBoardUpdate();
        //     }
        //     else if (mode == ControlMode.ROS)
        //     {
        //         ROSUpdate();
        //     }     
        // }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = 0;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }

//         private void KeyBoardUpdate()
//         {
//             float moveDirection = Input.GetAxis("Vertical");
//             float inputSpeed;
//             float inputRotationSpeed;
//             if (moveDirection > 0)
//             {
//                 inputSpeed = maxLinearSpeed;
//             }
//             else if (moveDirection < 0)
//             {
//                 inputSpeed = maxLinearSpeed * -1;
//             }
//             else
//             {
//                 inputSpeed = 0;
//             }

//             float turnDirction = Input.GetAxis("Horizontal");
//             if (turnDirction > 0)
//             {
//                 inputRotationSpeed = maxRotationalSpeed;
//             }
//             else if (turnDirction < 0)
//             {
//                 inputRotationSpeed = maxRotationalSpeed * -1;
//             }
//             else
//             {
//                 inputRotationSpeed = 0;
//             }
//             RobotInput(inputSpeed, inputRotationSpeed);
//         }


        // private void ROSUpdate()
        // {
        //     if (Time.time - lastCmdReceived > ROSTimeout)
        //     {
        //         SetSpeed(tA1, 0);
        //         SetSpeed(tA2, 0);
        //         SetSpeed(tA3, 0);
        //         SetSpeed(tA4, 0);
        //         SetSpeed(tA5, 0);
        //         SetSpeed(tA6, 0);
        //         SetSpeed(tA7, 0);
        //         SetSpeed(tA8, 0);
        //     }
        // }

//         private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
//         {
//             if (speed > maxLinearSpeed)
//             {
//                 speed = maxLinearSpeed;
//             }
//             if (rotSpeed > maxRotationalSpeed)
//             {
//                 rotSpeed = maxRotationalSpeed;
//             }
//             float wheel1Rotation = (speed / wheelRadius);
//             float wheel2Rotation = wheel1Rotation;
//             float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
//             if (rotSpeed != 0)
//             {
//                 wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
//                 wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
//             }
//             else
//             {
//                 wheel1Rotation *= Mathf.Rad2Deg;
//                 wheel2Rotation *= Mathf.Rad2Deg;
//             }
//             SetSpeed(wA1, wheel1Rotation);
//             SetSpeed(wA2, wheel2Rotation);
//         }
    }
}
