// ============================================================================
// AGVController.cs - Differential Drive Robot Controller
// ============================================================================
//
// This script controls the robot's wheels. It's the "muscles" of the robot.
//
// ============================================================================
// HOW DIFFERENTIAL DRIVE WORKS:
// ============================================================================
//
// A differential drive robot (like Turtlebot) has 2 wheels, one on each side.
// By spinning these wheels at different speeds, the robot can:
//
// 1. GO STRAIGHT: Both wheels spin at same speed
//    [Left: 100 rpm] [Right: 100 rpm] → Robot goes forward
//
// 2. TURN IN PLACE: Wheels spin opposite directions
//    [Left: +100 rpm] [Right: -100 rpm] → Robot spins clockwise
//
// 3. CURVE: Wheels spin at different speeds
//    [Left: 150 rpm] [Right: 50 rpm] → Robot curves to the right
//
// ============================================================================
// CONTROL MODES:
// ============================================================================
//
// This controller supports 3 input sources:
//
// 1. KEYBOARD MODE: Human presses W/A/S/D keys
// 2. ROS MODE: Commands come from ROS2 via /cmd_vel topic
// 3. ML MODE: Commands come from ML-Agents neural network
//
// All three modes eventually call RobotInput(linear, angular) which
// converts velocity commands to individual wheel speeds.
//
// ============================================================================
// THE MATH: Converting (linear, angular) to wheel speeds
// ============================================================================
//
// Given:
// - linear velocity V (m/s) - how fast to move forward
// - angular velocity ω (rad/s) - how fast to rotate
// - wheel radius r (m)
// - track width L (m) - distance between wheels
//
// Formulas:
// - left_wheel_speed  = (V + ω*L/2) / r
// - right_wheel_speed = (V - ω*L/2) / r
//
// Example: Move forward at 0.5 m/s while turning right at 0.5 rad/s
// - V = 0.5, ω = 0.5, r = 0.033, L = 0.288
// - left  = (0.5 + 0.5*0.144) / 0.033 = (0.5 + 0.072) / 0.033 = 17.3 rad/s
// - right = (0.5 - 0.072) / 0.033 = 13.0 rad/s
// - Left wheel spins faster → robot curves right ✓
//
// ============================================================================

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

namespace RosSharp.Control
{
    /// <summary>
    /// Control mode selection - determines where velocity commands come from.
    /// </summary>
    public enum ControlMode 
    { 
        Keyboard,  // Human control via W/A/S/D keys
        ROS,       // Commands from ROS2 navigation stack
        ML         // Commands from ML-Agents neural network
    }

    /// <summary>
    /// Differential drive controller for Turtlebot-style robots.
    /// Converts high-level velocity commands (linear, angular) into
    /// individual wheel rotation speeds.
    /// </summary>
    public class AGVController : MonoBehaviour
    {
        // ====================================================================
        // ML-AGENTS COMMANDS
        // ====================================================================
        // These store the latest commands from the neural network.
        // Updated by SetMLCommands(), applied in FixedUpdate().
        
        private float mlLinear = 0f;   // Forward/backward velocity from ML
        private float mlAngular = 0f;  // Turning velocity from ML
        
        // ====================================================================
        // WHEEL REFERENCES
        // ====================================================================
        // These point to the actual wheel GameObjects in the robot hierarchy.
        // Each wheel has an ArticulationBody component for physics simulation.
        
        [Header("Wheel GameObjects")]
        [Tooltip("Left wheel - usually wheel_left_link")]
        public GameObject wheel1;  // Left wheel
        
        [Tooltip("Right wheel - usually wheel_right_link")]
        public GameObject wheel2;  // Right wheel
        
        // ====================================================================
        // CONTROL MODE
        // ====================================================================
        
        [Header("Control Settings")]
        [Tooltip("Where do velocity commands come from?")]
        public ControlMode mode = ControlMode.ROS;

        // Physics components for wheels
        private ArticulationBody wA1;  // Left wheel ArticulationBody
        private ArticulationBody wA2;  // Right wheel ArticulationBody

        // ====================================================================
        // ROBOT PHYSICAL PARAMETERS
        // ====================================================================
        // These must match the actual robot model! Incorrect values cause
        // the robot to move wrong (too fast, wrong turn radius, etc.)
        
        [Header("Robot Physical Parameters")]
        [Tooltip("Maximum forward speed (m/s). Higher = faster robot.")]
        public float maxLinearSpeed = 2f;
        
        [Tooltip("Maximum turning speed (rad/s). Higher = faster spinning.")]
        public float maxRotationalSpeed = 1f;
        
        [Tooltip("Radius of each wheel in meters. Turtlebot3 = 0.033m")]
        public float wheelRadius = 0.033f;
        
        [Tooltip("Distance between left and right wheels in meters. Turtlebot3 = 0.288m")]
        public float trackWidth = 0.288f;
        
        [Tooltip("Maximum torque the wheel motors can apply")]
        public float forceLimit = 10f;
        
        [Tooltip("How quickly wheels slow down when not powered")]
        public float damping = 10f;

        // ====================================================================
        // ROS SETTINGS
        // ====================================================================
        
        [Header("ROS Settings")]
        [Tooltip("Stop robot if no ROS command received for this many seconds")]
        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;  // Time of last ROS message
        
        [Tooltip("ROS topic to listen for velocity commands")]
        public string cmdVelTopic = "/cmd_vel";

        // ROS connection and stored commands
        ROSConnection ros;
        private RotationDirection direction;
        private float rosLinear = 0f;   // Forward/backward velocity from ROS
        private float rosAngular = 0f;  // Turning velocity from ROS

        // ====================================================================
        // START - Initialize wheels and ROS connection
        // ====================================================================
        void Start()
        {
            // Get ArticulationBody components from wheel GameObjects
            wA1 = wheel1.GetComponent<ArticulationBody>();
            wA2 = wheel2.GetComponent<ArticulationBody>();
            
            // Configure wheel physics (force limits, damping)
            SetParameters(wA1);
            SetParameters(wA2);
            
            // Set up ROS connection for receiving velocity commands
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);
            ros.Subscribe<TwistMsg>(cmdVelTopic, ReceiveROSCmd);
        }

        // ====================================================================
        // SET ML COMMANDS - Called by TurtlebotExplore agent
        // ====================================================================
        /// <summary>
        /// Receives velocity commands from the ML-Agents neural network.
        /// Called by TurtlebotExplore.OnActionReceived() every decision step.
        /// 
        /// The angular velocity is NEGATED to match the ROS convention.
        /// In ROS: positive angular = turn left
        /// In our physics: positive angular = turn right
        /// So we negate to keep everything consistent.
        /// </summary>
        /// <param name="linear">Forward/backward velocity in m/s (positive = forward)</param>
        /// <param name="angular">Turning velocity in rad/s (positive = turn left)</param>
        public void SetMLCommands(float linear, float angular)
        {
            mlLinear = linear;
            
            // IMPORTANT: Negate angular to match ROS convention!
            // Without this, when the agent says "turn left", the robot turns right,
            // which completely breaks learning.
            mlAngular = -angular;
        }

        // ====================================================================
        // RECEIVE ROS COMMAND - Callback when ROS message arrives
        // ====================================================================
        /// <summary>
        /// Called automatically when a Twist message arrives on /cmd_vel.
        /// Stores the commanded velocities for use in next FixedUpdate.
        /// </summary>
        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinear = (float)cmdVel.linear.x;   // Forward/backward
            rosAngular = (float)cmdVel.angular.z; // Rotation around vertical axis
            lastCmdReceived = Time.time;          // Record when we got this
        }

        // ====================================================================
        // FIXED UPDATE - Apply motor commands to wheels
        // ====================================================================
        /// <summary>
        /// Called every physics step (typically 50 times per second).
        /// Reads commands from the current control mode and applies them
        /// to the wheel motors.
        /// </summary>
        void FixedUpdate()
        {
            // Dispatch based on current control mode
            if (mode == ControlMode.Keyboard)
            {
                KeyBoardUpdate();  // Read keyboard, call RobotInput
            }
            else if (mode == ControlMode.ROS)
            {
                ROSUpdate();       // Use ROS commands, call RobotInput
            }
            else if (mode == ControlMode.ML)
            {
                // Use ML commands directly
                // Note: mlAngular was already negated in SetMLCommands
                RobotInput(mlLinear, mlAngular);
            }     
        }

        // ====================================================================
        // SET PARAMETERS - Configure wheel physics
        // ====================================================================
        /// <summary>
        /// Configures the ArticulationBody drive settings for a wheel.
        /// These settings control how the wheel motor behaves.
        /// </summary>
        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;  // Max motor torque
            drive.damping = damping;        // Resistance to motion
            joint.xDrive = drive;
        }

        // ====================================================================
        // SET SPEED - Set a wheel's target rotation speed
        // ====================================================================
        /// <summary>
        /// Sets the target velocity for a wheel's motor.
        /// The ArticulationBody will try to reach this speed.
        /// </summary>
        /// <param name="joint">Which wheel to control</param>
        /// <param name="wheelSpeed">Target speed in degrees/second</param>
        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            
            if (float.IsNaN(wheelSpeed))
            {
                // Default behavior (not typically used)
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
            }
            else
            {
                // Set specific target velocity
                drive.targetVelocity = wheelSpeed;
            }
            
            joint.xDrive = drive;
        }

        // ====================================================================
        // KEYBOARD UPDATE - Read keyboard input
        // ====================================================================
        /// <summary>
        /// Reads W/A/S/D key input and converts to velocity commands.
        /// </summary>
        private void KeyBoardUpdate()
        {
            // Forward/backward from W/S keys
            float moveDirection = Input.GetAxis("Vertical");
            float inputSpeed;
            
            if (moveDirection > 0)
                inputSpeed = maxLinearSpeed;      // W pressed: go forward
            else if (moveDirection < 0)
                inputSpeed = -maxLinearSpeed;     // S pressed: go backward
            else
                inputSpeed = 0;                   // No key: stop

            // Left/right from A/D keys
            float turnDirection = Input.GetAxis("Horizontal");
            float inputRotationSpeed;
            
            if (turnDirection > 0)
                inputRotationSpeed = maxRotationalSpeed;   // D pressed: turn right
            else if (turnDirection < 0)
                inputRotationSpeed = -maxRotationalSpeed;  // A pressed: turn left
            else
                inputRotationSpeed = 0;                    // No key: no rotation

            RobotInput(inputSpeed, inputRotationSpeed);
        }

        // ====================================================================
        // ROS UPDATE - Apply ROS commands with timeout
        // ====================================================================
        /// <summary>
        /// Applies stored ROS velocity commands to the robot.
        /// Stops robot if no commands received recently (safety feature).
        /// </summary>
        private void ROSUpdate()
        {
            // Safety: Stop if no ROS command received recently
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }
            
            // Apply commands (note: angular is negated for convention)
            RobotInput(rosLinear, -rosAngular);
        }

        // ====================================================================
        // ROBOT INPUT - Convert velocities to wheel speeds (THE CORE MATH)
        // ====================================================================
        /// <summary>
        /// Converts high-level velocity commands into individual wheel speeds.
        /// This is where differential drive kinematics are applied.
        /// 
        /// The math:
        /// - Both wheels start at base speed: V / r (rad/s)
        /// - For turning, we add/subtract: ω * L / (2 * r)
        /// - Left wheel gets + (spins faster to turn right)
        /// - Right wheel gets - (spins slower to turn right)
        /// </summary>
        /// <param name="speed">Linear velocity in m/s (positive = forward)</param>
        /// <param name="rotSpeed">Angular velocity in rad/s (positive = turn right)</param>
        private void RobotInput(float speed, float rotSpeed)
        {
            // Clamp inputs to safe range (both positive AND negative)
            speed = Mathf.Clamp(speed, -maxLinearSpeed, maxLinearSpeed);
            rotSpeed = Mathf.Clamp(rotSpeed, -maxRotationalSpeed, maxRotationalSpeed);
            
            // ================================================================
            // DIFFERENTIAL DRIVE KINEMATICS
            // ================================================================
            // 
            // Base wheel rotation rate (rad/s) for straight motion:
            // wheel_speed = linear_velocity / wheel_radius
            //
            // For turning, we create a speed difference between wheels:
            // wheel_speed_diff = angular_velocity * track_width / wheel_radius
            //
            // Left wheel:  base_speed + (diff / 2)
            // Right wheel: base_speed - (diff / 2)
            //
            // If angular velocity is positive (turn right):
            // - Left wheel spins FASTER (positive diff)
            // - Right wheel spins SLOWER (negative diff)
            // - This makes the robot turn right ✓
            
            // Base rotation rate for both wheels (rad/s)
            float wheel1Rotation = speed / wheelRadius;
            float wheel2Rotation = wheel1Rotation;
            
            // Calculate wheel speed difference for turning
            float wheelSpeedDiff = (rotSpeed * trackWidth) / wheelRadius;
            
            if (rotSpeed != 0)
            {
                // Apply differential + convert to degrees/second for Unity
                wheel1Rotation = (wheel1Rotation + wheelSpeedDiff) * Mathf.Rad2Deg;
                wheel2Rotation = (wheel2Rotation - wheelSpeedDiff) * Mathf.Rad2Deg;
            }
            else
            {
                // Just convert to degrees/second
                wheel1Rotation *= Mathf.Rad2Deg;
                wheel2Rotation *= Mathf.Rad2Deg;
            }
            
            // Apply to wheel motors
            SetSpeed(wA1, wheel1Rotation);  // Left wheel
            SetSpeed(wA2, wheel2Rotation);  // Right wheel
        }
    }
}
