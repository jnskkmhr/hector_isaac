from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.timeline import get_timeline_interface
import omni.graph.core as og
import carb
import numpy as np
from pathlib import Path

from hector import Robot
from config import EnvironmentConfig, RobotConfig

enable_extension("omni.isaac.ros_bridge")
simulation_app.update()

# check if rosmaster node is running
import rosgraph
if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class HectorRunner:
    def __init__(self, env_cfg:EnvironmentConfig, robot_cfg:RobotConfig)->None:
        """_summary_

        Args:
            robot_cfg (RobotConfig): Configuration for the robot
            env_cfg (EnvironmentConfig): Configuration for the environment
        """
        
        self._world = World(stage_units_in_meters=1.0, physics_dt=env_cfg.physics_dt, rendering_dt=env_cfg.render_dt)
        self._timeline = get_timeline_interface()
        self._world.scene.add_default_ground_plane()
        self._hector = Robot(usd_path=robot_cfg.usd_path, prim_path=robot_cfg.prim_path, name=robot_cfg.name, position=robot_cfg.position)
        self._hector.init(self._world)
        self._world.reset()
        
        self.ctr_sim_remap = robot_cfg.ctr_sim_remap
        self._create_og()
        self._init_publisher()
        self._init_subscriber()
        self._init_msg()
        
    def _create_og(self):
        # Creating an ondemand push graph with ROS Clock, everything in the ROS environment must synchronize with this clock
        try:
            keys = og.Controller.Keys
            (self._clock_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": "/ROS_Clock",
                    "evaluator_name": "push",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("publishClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "publishClock.inputs:execIn"),
                        ("readSimTime.outputs:simulationTime", "publishClock.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)
            simulation_app.close()
            exit()
    
    def _init_publisher(self)->None:
        self._odom_publisher = rospy.Publisher('/hector/Odometry', Odometry, queue_size=21)
        self._joint_state_publisher = rospy.Publisher('/hector/llcontroller/state', JointState, queue_size=21)
    
    def _init_subscriber(self)->None:
        self._sub_joint_cmd = rospy.Subscriber(
            "/hector/llcontroller/command", Float32MultiArray, self.torque_command_callback
        )
        # buffer to store the robot command
        self._llcmd = np.zeros(len(self._hector.joint_names), dtype=np.float32)
    
    def _init_msg(self)->None:
        self._odom_msg = Odometry()
        self._odom_msg.header.frame_id = "odom"
        self._joint_state_msg = JointState()
        self._joint_state_msg.name = [self._hector.joint_names[i] for i in self.ctr_sim_remap]
    
    def setup(self)->None:
        """
        [Summary]

        add physics callback

        """
        self._world.add_physics_callback("robot_sim_step", callback_fn=self.robot_simulation_step)

    def run(self)->None:
        """
        [Summary]

        Step simulation based on rendering downtime

        """
        self._timeline.play()
        while simulation_app.is_running():
            self._world.step(render=True)
    
    def publish_ros_data(self)->None:
        """
        [Summary]

        Publish body pose, joint state, imu data

        """
        # update all header timestamps
        ros_timestamp = rospy.get_rostime()
        self._odom_msg.header.stamp = ros_timestamp
        self._joint_state_msg.header.stamp = ros_timestamp

        # a) body odometry
        self._update_odometry()
        self._odom_publisher.publish(self._odom_msg)
        # b) joint state
        self._update_joint_state()
        self._joint_state_publisher.publish(self._joint_state_msg)
    
    def robot_simulation_step(self, step_size):
        """
        [Summary]

        Tick ros bridge and publish ros data

        """
        
        # Tick the ROS Clock
        og.Controller.evaluate_sync(self._clock_graph)

        # Publish ROS data
        self.publish_ros_data()
    
    def _update_odometry(self)->None:
        """
        [Summary]

        Update odometry message

        """
        position, orientation, velocity, angular_velocity = self._hector.get_body_state()
        self._odom_msg.pose.pose.position.x = position[0]
        self._odom_msg.pose.pose.position.y = position[1]
        self._odom_msg.pose.pose.position.z = position[2]
        self._odom_msg.pose.pose.orientation.x = orientation[0]
        self._odom_msg.pose.pose.orientation.y = orientation[1]
        self._odom_msg.pose.pose.orientation.z = orientation[2]
        self._odom_msg.pose.pose.orientation.w = orientation[3]
        self._odom_msg.twist.twist.linear.x = velocity[0]
        self._odom_msg.twist.twist.linear.y = velocity[1]
        self._odom_msg.twist.twist.linear.z = velocity[2]
        self._odom_msg.twist.twist.angular.x = angular_velocity[0]
        self._odom_msg.twist.twist.angular.y = angular_velocity[1]
        self._odom_msg.twist.twist.angular.z = angular_velocity[2]
    
    def _update_joint_state(self)->None:
        """
        [Summary]

        Update joint state message

        """
        joint_position, joint_velocity, joint_effort_est = self._hector.get_joint_state()
        self._joint_state_msg.position = joint_position[self.ctr_sim_remap]
        self._joint_state_msg.velocity = joint_velocity[self.ctr_sim_remap]
        self._joint_state_msg.effort = joint_effort_est[self.ctr_sim_remap]
    
    def torque_command_callback(self, data:Float32MultiArray)->None:
        """
        [Summary]

        Joint command call back, set command torque for the joints

        """
        for i in range(len(self._hector.joint_names)):
            self._llcmd[self.ctr_sim_remap[i]] = data[i]
        self._hector.apply_effort(self._llcmd)
        print("Command received", self._llcmd)


def main():
    """
    [Summary]

    The function launches the simulator, creates the robot, and run the simulation steps

    """
    # first enable ros node, make sure using simulation time
    rospy.init_node("isaac_hector", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
    rospy.set_param("use_sim_time", True)
    physics_dt = 1/1000
    render_dt = 1/60
    
    # control node: ["L_hip", "L_hip2", "L_thigh", "L_calf", "L_toe", "R_hip", "R_hip2", "R_thigh", "R_calf", "R_toe"]
    # isaac: ['L_hip', 'R_hip', 'L_hip2', 'R_hip2', 'L_thigh', 'R_thigh', 'L_calf', 'R_calf', 'L_toe', 'R_toe']
    env_cfg = EnvironmentConfig(physics_dt=physics_dt, render_dt=render_dt)
    robot_cfg = RobotConfig(
        usd_path=str(Path(__file__).parent / "asset/hector_ros1.usd"),   
        prim_path="/World/hector", 
        name="hector", 
        position=[0, 0, 0.55], 
        ctr_sim_remap=[0, 5, 1, 6, 2, 7, 3, 8, 4, 9] # control index to simulation index (vise versa)
        )
    
    runner = HectorRunner(env_cfg, robot_cfg)
    simulation_app.update()
    runner.setup()

    # an extra reset is needed to register
    runner._world.reset()
    runner.run()
    rospy.signal_shutdown("simulation complete")
    simulation_app.close()


if __name__ == "__main__":
    main()