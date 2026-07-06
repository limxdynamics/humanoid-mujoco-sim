# Copyright information
#
# © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

import os
import sys
import time
import subprocess
import atexit
import mujoco
import mujoco.viewer as viewer
from functools import partial
import limxsdk
import limxsdk.robot.Rate as Rate
import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes

class SimulatorMujoco:
    def __init__(self, asset_path, robot, floating_base): 
        self.robot = robot
        self.floating_base = floating_base
        
        # Load the MuJoCo model and data from the specified XML asset path
        self.mujoco_model = mujoco.MjModel.from_xml_path(asset_path)
        self.mujoco_data = mujoco.MjData(self.mujoco_model)

        # Get the number of actuators
        self.actuator_count = self.mujoco_model.nu

        # Get the number of joints
        self.joint_num = self.mujoco_model.njnt

        # Get the joint names
        self.joint_sensor_names = []
        for i in range(self.joint_num):
            joint_name = mujoco.mj_id2name(self.mujoco_model, mujoco.mjtObj.mjOBJ_JOINT, i)
            self.joint_sensor_names.append(joint_name)
        
        # Launch the MuJoCo viewer in passive mode with custom settings
        self.viewer = viewer.launch_passive(self.mujoco_model, self.mujoco_data, key_callback=self.key_callback, show_left_ui=True, show_right_ui=True)
        self.viewer.cam.distance = 10  # Set camera distance
        self.viewer.cam.elevation = -20  # Set camera elevation
    
        self.dt = self.mujoco_model.opt.timestep  # Get simulation timestep
        self.fps = 1 / self.dt  # Calculate frames per second (FPS)

        # Initialize robot command data with default values
        self.robot_cmd = datatypes.RobotCmd()
        self.robot_cmd.mode = [0. for x in range(0, self.actuator_count)]
        self.robot_cmd.q = [0. for x in range(0, self.actuator_count)]
        self.robot_cmd.dq = [0. for x in range(0, self.actuator_count)]
        self.robot_cmd.tau = [0. for x in range(0, self.actuator_count)]
        self.robot_cmd.Kp = [0. for x in range(0, self.actuator_count)]
        self.robot_cmd.Kd = [0. for x in range(0, self.actuator_count)]

        # Initialize robot state data with default values
        self.robot_state = datatypes.RobotState()
        self.robot_state.tau = [0. for x in range(0, self.actuator_count)]
        self.robot_state.q = [0. for x in range(0, self.actuator_count)]
        self.robot_state.dq = [0. for x in range(0, self.actuator_count)]
        self.robot_state.motor_names = ['' for x in range(0, self.actuator_count)]

        # Initialize IMU data structure
        self.imu_data = datatypes.ImuData()

        # Set up callback for receiving robot commands in simulation mode
        self.robotCmdCallbackPartial = partial(self.robotCmdCallback)
        self.robot.subscribeRobotCmdForSim(self.robotCmdCallbackPartial)

    # Callback function for receiving robot command data
    def robotCmdCallback(self, robot_cmd: datatypes.RobotCmd):
        self.robot_cmd = robot_cmd

    # Callback for keypress events in the MuJoCo viewer (currently does nothing)
    def key_callback(self, keycode):
        pass

    def run(self):
        frame_count = 0
        self.rate = Rate(self.fps)  # Set the update rate according to FPS
        while self.viewer.is_running():    
            # Step the MuJoCo physics simulation
            mujoco.mj_step(self.mujoco_model, self.mujoco_data)
            if not self.floating_base:
                # Extract IMU data (orientation, gyro, and acceleration) from simulation
                self.imu_data.quat[0] = self.mujoco_data.sensordata[0]
                self.imu_data.quat[1] = self.mujoco_data.sensordata[1]
                self.imu_data.quat[2] = self.mujoco_data.sensordata[2]
                self.imu_data.quat[3] = self.mujoco_data.sensordata[3]

                self.imu_data.gyro[0] = self.mujoco_data.sensordata[4]
                self.imu_data.gyro[1] = self.mujoco_data.sensordata[5]
                self.imu_data.gyro[2] = self.mujoco_data.sensordata[6]

                self.imu_data.acc[0] = self.mujoco_data.sensordata[7]
                self.imu_data.acc[1] = self.mujoco_data.sensordata[8]
                self.imu_data.acc[2] = self.mujoco_data.sensordata[9]

                # Set the timestamp for the current IMU data and publish it
                self.imu_data.stamp = time.time_ns()
                self.robot.publishImuDataForSim(self.imu_data)

                # Update robot state data from simulation
                for i in range(self.actuator_count):
                    self.robot_state.q[i] = self.mujoco_data.sensordata[i + 10]
                    self.robot_state.dq[i] = self.mujoco_data.sensordata[self.actuator_count + i + 10]
                    self.robot_state.tau[i] = self.mujoco_data.ctrl[i]

                    # Apply control commands to the robot based on the received robot command data
                    self.mujoco_data.ctrl[i] = (
                        self.robot_cmd.Kp[i] * (self.robot_cmd.q[i] - self.robot_state.q[i]) + 
                        self.robot_cmd.Kd[i] * (self.robot_cmd.dq[i] - self.robot_state.dq[i]) + 
                        self.robot_cmd.tau[i]
                    )
            else:
                # Update robot state data from simulation
                for i in range(self.actuator_count):
                    self.robot_state.q[i] = self.mujoco_data.sensordata[i]
                    self.robot_state.dq[i] = self.mujoco_data.sensordata[self.actuator_count + i]
                    self.robot_state.tau[i] = self.mujoco_data.ctrl[i]

                    # Apply control commands to the robot based on the received robot command data
                    self.mujoco_data.ctrl[i] = (
                        self.robot_cmd.Kp[i] * (self.robot_cmd.q[i] - self.robot_state.q[i]) + 
                        self.robot_cmd.Kd[i] * (self.robot_cmd.dq[i] - self.robot_state.dq[i]) + 
                        self.robot_cmd.tau[i]
                    )
        
            # Set the timestamp for the current robot state and publish it
            self.robot_state.stamp = time.time_ns()
            self.robot.publishRobotStateForSim(self.robot_state)

            # Sync the viewer every 20 frames for smoother visualization
            if frame_count % 20 == 0:
                self.viewer.sync()

            frame_count += 1
            self.rate.sleep()  # Maintain the simulation loop at the correct rate

def run_kinematic_projection():
    try:
        # Get the directory where the current Python script is located
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # Build the full path of the executable program
        program_path = os.path.join(current_dir, 'prebuild/kinematic_projection')
        
        # Build the full path of the executable program etc
        program_etc_path = os.path.join(current_dir, 'prebuild/etc')

        # Copy the current environment variables
        env = os.environ.copy()
        env['MROS_ETC_PATH'] = program_etc_path
        env['MROS_LOG_LEVEL'] = "0"

        # Start the executable program and pass in the modified environment variables
        process = subprocess.Popen(program_path, env=env)

        def cleanup():
            # When the Python script exits, check if the subprocess is still running
            if process.poll() is None:
                print("Trying to terminate the subprocess...")
                # Send a termination signal to the subprocess
                process.terminate()
                try:
                    # Wait for the subprocess to terminate within 5 seconds
                    process.wait(timeout=5)
                    print("The subprocess has been successfully terminated.")
                except subprocess.TimeoutExpired:
                    # If it times out, force kill the subprocess
                    print("The subprocess did not terminate within the specified time, forcing termination...")
                    process.kill()
                    print("The subprocess has been force-killed.")

        # Register the cleanup function to ensure it is called when the script exits
        atexit.register(cleanup)
        return process
    except FileNotFoundError:
        print("Error: The executable program was not found. Please make sure kinematic_projection is in the current directory.")
    except PermissionError:
        print("Error: You do not have permission to execute the program.")
    except Exception as e:
        print(f"An unknown error occurred: {e}")
    return None

if __name__ == '__main__': 
    robot_type = os.getenv("ROBOT_TYPE")

    # Check if the ROBOT_TYPE environment variable is set, otherwise exit with an error
    if not robot_type:
        print("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.")
        sys.exit(1)
        
    # Split from the right side of ROBOT_TYPE and get the content before the first underscore as the main type
    main_robot_type = robot_type.rsplit('_', 1)[0]

    # Is floating base
    floating_base = main_robot_type.startswith(('DA_', 'UB_'))

    # Default IP address for the robot
    robot_ip = "127.0.0.1"
    
    # Check if command-line argument is provided for robot IP
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]
    
    ip_parts = robot_ip.split('.')
    if len(ip_parts) == 4:
        os.environ["MROS_IP_LIST"] = f"{ip_parts[0]}.{ip_parts[1]}.{ip_parts[2]}.x"
    else:
        print(f"ERROR: Invalid IPv4 format ({robot_ip})")
        sys.exit(1)

    # Create a Robot instance of the Humanoid type
    robot = Robot(RobotType.Humanoid, True)

    # Initialize the robot with the provided IP address
    if not robot.init(robot_ip):
        sys.exit(1)

    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Define the path to the robot model XML file based on the robot type
    model_path = f'{script_dir}/humanoid-description/{main_robot_type}_description/xml/{robot_type}.xml'

    # Check if the model file exists, otherwise exit with an error
    if not os.path.exists(model_path):
        print(f"Error: The file {model_path} does not exist. Please ensure the ROBOT_TYPE is set correctly.")
        sys.exit(1)
    
    # Run kinematic_projection
    run_kinematic_projection()

    # Create and run the MuJoCo simulator instance
    print(f"*** Model File Loaded: humanoid-description/{main_robot_type}_description/xml/{robot_type}.xml ***")
    simulator = SimulatorMujoco(model_path, robot, floating_base)
    simulator.run()
