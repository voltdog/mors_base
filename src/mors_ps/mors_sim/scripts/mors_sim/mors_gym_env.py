"""
This file implements the functionalities of a Mors Gym environment using pybullet physical engine.
"""
import copy
import numpy as np
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import time
import random
import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils import seeding
import pybullet
from pybullet_utils import bullet_client as bc
import pybullet_data
from pkg_resources import parse_version
from mors_sim.motor_simple import MotorSimple
from mors_sim.motor_accurate import MotorAccurate
from mors_sim.world_creator import WorldCreator
from mors_sim.lidar import Lidar
from mors_sim.camera import Camera

import math

INIT_POSITION = [0, 0, .76]
INIT_ORIENTATION = [0, 0, 0, 1]
LEG_POSITION = ["R1", "L1", "R2", "L2"]
MOTOR_NAMES = [
    "abad_joint_R1", "hip_joint_R1", "knee_joint_R1",
    "abad_joint_L1", "hip_joint_L1", "knee_joint_L1",
    "abad_joint_R2", "hip_joint_R2", "knee_joint_R2",
    "abad_joint_L2", "hip_joint_L2", "knee_joint_L2",
]
LEG_LINK_ID = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
MOTOR_LINK_ID = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11] # shoulders: 0, 3, 6, 9
SHOULDER_MOTOR_ID = [0, 3, 6, 9]
FOOT_LINK_ID = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] # it is needed for friction setting
BASE_LINK_ID = -1

NUM_SUBSTEPS = 5
NUM_MOTORS = 12
MOTOR_ANGLE_OBSERVATION_INDEX = 0
MOTOR_VELOCITY_OBSERVATION_INDEX = MOTOR_ANGLE_OBSERVATION_INDEX + NUM_MOTORS
MOTOR_TORQUE_OBSERVATION_INDEX = MOTOR_VELOCITY_OBSERVATION_INDEX + NUM_MOTORS
BASE_ORIENTATION_OBSERVATION_INDEX = MOTOR_TORQUE_OBSERVATION_INDEX + NUM_MOTORS
ACTION_EPS = 0.01
OBSERVATION_EPS = 0.01
RENDER_HEIGHT = 720
RENDER_WIDTH = 960
K_SCALE = 2

MOTOR_ANGLE_MAX = 2.62
MOTOR_VELOCITY_MAX = 12
MOTOR_TORQUE_MAX = 110
BASE_ORIENTATION_MAX = 1.57

# LIDAR_JOINT = 27

class MorsBulletEnv(gym.Env):
  """
  The gym environment for MORS.

  It simulates the locomotion of a mors, a quadruped robot. The state space
  include base orientation, the angles, velocities and torques for all the motors and the action
  space is the desired motor angle for each motor. The reward function is based
  on how fast robot can move without twitches and side deviations.

  """
  metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 50}

  def __init__(self,
              urdf_root="./urdf",
              sim_freq=100,
              world="empty",
              hard_reset=False,
              render=True,
              on_rack=False,
              self_collision_enabled=False,
              debug_mode = False,
              floating_camera = False,
              step_enabled = True,

              accurate_motor_model_enabled=False,
              simple_motor_model_enabled=False,
              torque_control_enabled=False,
              motor_kp=30.0,
              motor_kd=0.1,
              motor_velocity_limit=np.inf,
              
              max_timesteps = np.inf,
              action_repeat=1,
              rew_scale = 1,
              distance_limit=float("inf"),
              observation_noise_stdev=0.0,
              normalization=True,

              camera_enabled=False,
              lidar_enabled=False,
              ext_disturbance_enabled=False,

              ):

    """Initialize the mors gym environment.

    Args:
      urdf_root: The path to the urdf data folder relative to parent folder.
      sim_freq: The value that used to simulate environment.
      hard_reset: Whether to wipe the simulation and load everything when reset
        is called. If set to false, reset just place the mors back to start
        position and set its pose to initial configuration.
      render: Whether to render the simulation.
      on_rack: Whether to place the mors on rack. This is only used to debug
        walking gaits. In this mode, the mors's base is hanged midair so
        that its walking gait is clearer to visualize.
      self_collision_enabled: Whether to enable self collision in the sim.  
      max_timesteps: Time limit during which the simulation is going on.
      debug_mode: Whether to show side panel with debug parameters or not.
      floating_camera: If True, camera follows the robot.
      step_enabled: If True, it is needed to send motor commands as arguments in 
        step() function. If False, it is neede to use set_motor_commands() function. 
      accurate_motor_model_enabled: Whether to use the accurate BLDC motor model.
      simple_motor_model_enabled: Whether to use the simple version of BLDC motor model.
      torque_control_enabled: Whether to use the torque control, if set to
        False, pose control will be used.
      motor_kp: proportional gain for the accurate and simple motor models.
      motor_kd: derivative gain for the accurate and simple motor models.
      motor_velocity_limit: The velocity limit of each motor.
      
      action_repeat: The number of simulation steps before actions are applied.
      rew_scale: How much to gain reward. Sometimes it helps to learn a model.
      distance_limit: The maximum distance to terminate the episode.
      observation_noise_stdev: The standard deviation of observation noise.
      normalization: Whether to normalize observations or not.

          """
    self._time_step = 1/sim_freq
    self._num_bullet_solver_iterations = sim_freq*NUM_SUBSTEPS
    self._action_repeat = action_repeat
    self._urdf_root = urdf_root
    self._self_collision_enabled = self_collision_enabled
    self._motor_velocity_limit = motor_velocity_limit
    self._is_render = render
    self._world_name = world
    self._step_enabled = step_enabled
    self._floating_camera = floating_camera
    self._distance_limit = distance_limit
    self._observation_noise_stdev = observation_noise_stdev
    self._accurate_motor_model_enabled = accurate_motor_model_enabled
    self._simple_motor_model_enabled = simple_motor_model_enabled
    self._motor_kp = motor_kp
    self._motor_kd = motor_kd
    self._torque_control_enabled = torque_control_enabled
    self._on_rack = on_rack
    self._max_timesteps = max_timesteps
    self._rew_scale = rew_scale
    self._hard_reset = True
    self._debug_mode = debug_mode
    self._normalization = normalization
    self._ext_disturbance_enabled = ext_disturbance_enabled
    self._lidar_enabled = lidar_enabled
    self._camera_enabled = camera_enabled
    self.force = 136

    self._lateral_friction = 1.0
    self._spinning_friction = 0.0063

    self._ext_force_magn = 1200
    self._ext_force_duration = 6
    self._ext_force_interval = 2000
    self._force_dir = 1

    self._env_step_counter = 0
    self._action_bound = 1

    self._cam_dist = 1.0
    self._cam_yaw = 0
    self._cam_pitch = -30
    self._last_frame_time = 0.0

    self.num_legs = 4
    self.num_motors = int(self.num_legs * 3)
    self._motor_direction = [1]*12
    self._observed_motor_torques = np.zeros(self.num_motors)
    self._applied_motor_torques = np.zeros(self.num_motors)
    self._max_force = 3.5

    self.motor_angles = np.array([0]*12)
    self.motor_velocities = np.array([0]*12)
    self.motor_torques = np.array([0]*12)
    self.base_orientation = np.array([0]*3)

    # reward terms
    self.k_vel_x = 48
    self.k_body = 2
    self.k_vel_y = 3
    self.k_smooth = 0.2
    self.k_torq = 0.0003

    # motor models need smaller time step for stability.
    if self._simple_motor_model_enabled or self._accurate_motor_model_enabled:
      self._time_step /= NUM_SUBSTEPS
      self._num_bullet_solver_iterations /= NUM_SUBSTEPS
      self._action_repeat *= NUM_SUBSTEPS

    # create motor model objects
    if self._accurate_motor_model_enabled:
      self._accurate_motor_model = MotorAccurate(NUM_MOTORS)
      if self._torque_control_enabled:
        self._accurate_motor_model.set_kp(0)
        self._accurate_motor_model.set_kd(self._motor_kd)
      else:
        self._accurate_motor_model.set_kp(self._motor_kp)
        self._accurate_motor_model.set_kd(self._motor_kd)
    elif self._simple_motor_model_enabled:
      self._simple_motor_model = MotorSimple(NUM_MOTORS)
      if self._torque_control_enabled:
        self._simple_motor_model.set_kp(0)
        self._simple_motor_model.set_kd(self._motor_kd)
      else:
        self._simple_motor_model.set_kp(self._motor_kp)
        self._simple_motor_model.set_kd(self._motor_kd)


    if self._is_render:
      self._pybullet_client = bc.BulletClient(connection_mode=pybullet.GUI)
    else:
      self._pybullet_client = bc.BulletClient()
    self._pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())

    self.action_dim = NUM_MOTORS
    action_high = np.array([self._action_bound] * self.action_dim)
    self.action_space = spaces.Box(-np.float32(action_high), np.float32(action_high), dtype=np.float32)
    self.action = np.array([0]*self.action_dim, dtype=np.float32)
    self.action_prev = np.array([0]*self.action_dim, dtype=np.float32)
    self.action_space = spaces.Box(low=np.float32(-1), high=np.float32(1), shape=(self.action_dim,), dtype=np.float32)

    self.obs_dim = 39
    self.obs = np.array([0]*self.obs_dim, dtype=np.float32)
    observation_high = (self.get_observation_upper_bound())
    observation_low = (self.get_observation_lower_bound())
    self.observation_space = spaces.Box(np.float32(observation_low), np.float32(observation_high), dtype=np.float32)

    self.world_creator = WorldCreator(self._pybullet_client, spinning_friction=self._spinning_friction, lateral_friction=self._lateral_friction)
    
    self.set_friction()
    self.seed()
    self.reset()

    if self._lidar_enabled == True:
      self.lidar =  Lidar(pybullet_client=self._pybullet_client, 
                          robot_id=self.quadruped, 
                          joint_id=18,
                          frequency=20, 
                          base_frequency=sim_freq, 
                          render=True,
                          
                          angle_min=np.radians(130),
                          angle_max=-np.radians(130),
                          point_num=360,
                          range_min=0.25,
                          range_max=8.0, 
                          )
                
    if self._camera_enabled == True:
      self.camera = Camera(pybullet_client=self._pybullet_client, 
                           robot_id=self.quadruped, 
                           joint_id=17,
                           frequency=20, 
                           base_frequency=sim_freq)

    self._hard_reset = hard_reset # this needs to be after reset

    self.viewer = None
    
    self.fallen = 0

    # debug parameters
    self.view = True
    if self._debug_mode == True and self._is_render == True:
      self.reset_id = self._pybullet_client.addUserDebugParameter("Reset",1,0,1)
      self.view_id = self._pybullet_client.addUserDebugParameter("View",1,0,1)
      self.reset_clicked_prev = self._pybullet_client.readUserDebugParameter(self.reset_id)
      self.reset_clicked = self._pybullet_client.readUserDebugParameter(self.reset_id)
      self.view_clicked_prev = self._pybullet_client.readUserDebugParameter(self.view_id)
      self.view_clicked = self._pybullet_client.readUserDebugParameter(self.view_id)
      
      self.distance_id = self._pybullet_client.addUserDebugParameter("Distance",0.3, 2, 1.2)
      self.yaw_id = self._pybullet_client.addUserDebugParameter("Yaw",-180, 180, 47)
      self.pitch_id = self._pybullet_client.addUserDebugParameter("Pitch",-180, 180, -20)

      self.sl_id = self._pybullet_client.addUserDebugParameter("Step Length", 0, 0.2, 0)
      self.sh_id = self._pybullet_client.addUserDebugParameter("Step Height", 0, 0.1, 0)
      self.omega_id = self._pybullet_client.addUserDebugParameter("Step Frequency", 0, 4, 2.9)
      self.psi_id = self._pybullet_client.addUserDebugParameter("Sidewalk Angle", -3.14, 3.14, 0)
      self.rot_dir_id = self._pybullet_client.addUserDebugParameter("Turning Direction", -1.0, 1.0, 0.0)
      self.rot_r_id = self._pybullet_client.addUserDebugParameter("Turning Radius", 0, 10, 5)

      self.body_tr_x_id = self._pybullet_client.addUserDebugParameter("Body Translation X", -0.0003, 0.0003, 0)
      self.body_tr_y_id = self._pybullet_client.addUserDebugParameter("Body Translation Y", -0.0003, 0.0003, 0)
      self.body_tr_z_id = self._pybullet_client.addUserDebugParameter("Body Translation Z", -0.0003, 0.0003, 0)

      self.body_rot_x_id = self._pybullet_client.addUserDebugParameter("Body Rotation X", -0.5, 0.5, 0)
      self.body_rot_y_id = self._pybullet_client.addUserDebugParameter("Body Rotation Y", -0.4, 0.4, 0)
      self.body_rot_z_id = self._pybullet_client.addUserDebugParameter("Body Rotation Z", -0.4, 0.4, 0)

    self.first = True
      

  def reset(self, seed=0):
    """
    Resets all Mors parameters. 
    If hard_reset is True, it also resets simulation.
    """
    if self._hard_reset:
      self._pybullet_client.resetSimulation()
      self._pybullet_client.setPhysicsEngineParameter(
          numSolverIterations=int(self._num_bullet_solver_iterations))
      self._pybullet_client.setTimeStep(self._time_step)
      
      # plane = self._pybullet_client.loadURDF("%s/plane.urdf" % pybullet_data.getDataPath())
      # self._pybullet_client.changeVisualShape(plane, -1, rgbaColor=[1, 1, 1, 0.9])
      # self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_PLANAR_REFLECTION, 0)
      self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_SHADOWS, 0)
      self._pybullet_client.setGravity(0, 0, -9.81)
      # self.set_world(self._world_name)
      self.world_creator.create_world(self._world_name, lateralFriction=self._lateral_friction, spinningFriction=self._spinning_friction)
      self._reset_robot(reload_urdf=True)
    else:
      self._reset_robot(reload_urdf=False)

    self._env_step_counter = 0
    # self._last_base_position = np.array([0, 0, 0])
    # self._objectives = []
    self._pybullet_client.resetDebugVisualizerCamera(self._cam_dist, self._cam_yaw,
                                                     self._cam_pitch, [0, 0, 0])

    self.obs = self._noisy_observation()
    self.obs = self.norm_obs(self.obs)

    
    return self.obs,  {"prob": 1}
  
  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def step(self, action):
    """Step forward the simulation, given the action.

    Args:
      action: A list of desired motor angles for eight motors.

    Returns:
      observations: The angles, velocities and torques of all motors.
      reward: The reward for the current state-action pair.
      done: Whether the episode has ended.
      info: A dictionary that stores diagnostic information.

    Raises:
      ValueError: The action dimension is not the same as the number of motors.
      ValueError: The magnitude of actions is out of bounds.
    """
    # debug parameters
    
      
    if self._is_render:
      if self._debug_mode == True:
        # reset button
        self.reset_clicked = self._pybullet_client.readUserDebugParameter(self.reset_id)
        if self.reset_clicked != self.reset_clicked_prev:
            self._reset_robot(reload_urdf=False)
        self.reset_clicked_prev = self.reset_clicked
        # view button
        self.view_clicked = self._pybullet_client.readUserDebugParameter(self.view_id)
        if self.view_clicked != self.view_clicked_prev:
            self.view = not self.view
        self.view_clicked_prev = self.view_clicked
        # camera position
        distance = self._pybullet_client.readUserDebugParameter(self.distance_id)
        pitch = self._pybullet_client.readUserDebugParameter(self.pitch_id)
        yaw = self._pybullet_client.readUserDebugParameter(self.yaw_id)
      else:
        distance = 0.5
        yaw = -10
        pitch = -20
      # Sleep, otherwise the computation takes less time than real time,
      # which will make the visualization like a fast-forward video.
      time_spent = time.time() - self._last_frame_time
      self._last_frame_time = time.time()
      time_to_sleep = self._action_repeat * self._time_step - time_spent
      if time_to_sleep > 0:
        time.sleep(time_to_sleep)
      base_pos, _ = self.get_base_position_and_orientation()
      if self.view == True and self._floating_camera == False:
        self._pybullet_client.resetDebugVisualizerCamera(distance, yaw, pitch, base_pos)

    if self._ext_disturbance_enabled:
      self._apply_force()

    if self._lidar_enabled:
      self.lidar.update(self._env_step_counter)
    # if self._camera_enabled == True:
    #   self.camera.update(self._env_step_counter)

    self.action = action
    for _ in range(self._action_repeat):
      self._apply_action(self.action)
      self._pybullet_client.stepSimulation()

    self._env_step_counter += 1
    reward = self._rew_scale*self._reward()
    done = self._termination()
    self.obs = self._noisy_observation()
    if self._normalization == True:
      self.obs = self.norm_obs(self.obs)
    self.action_prev[:] = self.action[:]
    return self.obs, reward, done, False, {}
  
  def render(self, mode="rgb_array", close=False):
    if mode != "rgb_array":
      return np.array([])
    base_pos, _ = self.get_base_position_and_orientation()
    view_matrix = self._pybullet_client.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=base_pos,
        distance=self._cam_dist,
        yaw=self._cam_yaw,
        pitch=self._cam_pitch,
        roll=0,
        upAxisIndex=2)
    proj_matrix = self._pybullet_client.computeProjectionMatrixFOV(fov=60,
                                                                   aspect=float(RENDER_WIDTH) /
                                                                   RENDER_HEIGHT,
                                                                   nearVal=0.1,
                                                                   farVal=100.0)
    (_, _, px, _,
     _) = self._pybullet_client.getCameraImage(width=RENDER_WIDTH,
                                               height=RENDER_HEIGHT,
                                               viewMatrix=view_matrix,
                                               projectionMatrix=proj_matrix,
                                               renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
    rgb_array = np.array(px)
    rgb_array = rgb_array[:, :, :3]
    return rgb_array

  def close(self):
    """
    Close the environment and disconnect physical engine.
    It is recommended to use this function at the end of your program.
    """
    super().close()
    self._pybullet_client.disconnect()

  def is_fallen(self):
    """Decide whether the mors has fallen.

    If the up directions between the base and the world is larger (the dot
    product is smaller than 0.85) or the base is very low on the ground
    (the height is smaller than 0.13 meter), the mors is considered fallen.

    Returns:
      Boolean value that indicates whether the mors has fallen.
    """
    orientation = self.get_base_orientation()
    rot_mat = self._pybullet_client.getMatrixFromQuaternion(orientation)
    local_up = rot_mat[6:]
    pos, _ = self.get_base_position_and_orientation()
    return (np.dot(np.asarray([0, 0, 1]), np.asarray(local_up)) < 0.5 or pos[2] < 0.05)

  def _termination(self):
    position, _ = self.get_base_position_and_orientation()
    distance = np.sqrt(position[0]**2 + position[1]**2)
    if self.is_fallen():
      self.fallen = 1
    else:
      self.fallen = 0
    if self._env_step_counter >= self._max_timesteps:
      self.timeout = 1
    else:
      self.timeout = 0
    return bool(self.timeout or self.fallen or (distance > self._distance_limit))

  def _reward(self):
    """
    Calculate reward function
    Returns:
      reward value during current timestep
    """
    orient = np.abs(self.get_base_orientation())
    ang_vel = self.get_base_ang_vel()
    lin_vel = self.get_base_lin_vel()
    d_theta_ref_offset = self.action[0:12] - self.action_prev[0:12]

    self.vel_x_reward = lin_vel[0]**2
    self.body_reward = np.exp(-5*np.abs(ang_vel[0])) + np.exp(-5*np.abs(ang_vel[1])) + np.exp(-40*orient[0]) + np.exp(-40*orient[1])
    self.vel_y_reward = np.exp(-(lin_vel[1])**2/0.005)
    self.smooth_reward = np.exp(-np.linalg.norm(self.action[0:12], 1)) + np.exp(-np.linalg.norm(d_theta_ref_offset, 1))
    self.torque_penalty = np.linalg.norm(self.get_motor_torques()[:], 1)**2

    reward = (self.k_vel_x*self.vel_x_reward 
              + self.k_vel_y *self.vel_y_reward
              + self.k_body*self.body_reward 
              + self.k_smooth*self.smooth_reward
              - self.k_torq*self.torque_penalty)
    return reward
  
  def set_reward_terms(self, k_vel_x=48, k_vel_y=3, k_body=2, k_smooth=0.2, k_torq=0.0003):
    self.k_vel_x = k_vel_x
    self.k_vel_y = k_vel_y
    self.k_body = k_body
    self.k_smooth = k_smooth
    self.k_torq = k_torq
  
  def get_reward_parts(self):
    return [self.k_vel_x*self.vel_x_reward , self.k_vel_y *self.vel_y_reward, 
            self.k_body*self.body_reward, self.k_smooth*self.smooth_reward, 
            -self.k_torq*self.torque_penalty]
  
  def _noisy_observation(self):
    observation = self._get_observation()
    obs = np.array(observation, dtype=np.float32)
    if self._observation_noise_stdev > 0:
      obs += np.random.normal(scale=self._observation_noise_stdev, size=obs.shape)
    return obs

  def _get_observation(self):
    """Get the observations of mors.

    It includes the angles, velocities, torques and the orientation of the base.

    Returns:
      The observation list
      observation[0:11] are motor angles
      observation[12:23] are motor velocities
      observation[24:35] are motor torques
      observation[35:38] is the orientation of the base, in Euler form.
    """
    observation = []
    self.motor_angles = self.get_motor_angles().tolist() 
    self.motor_velocities = self.get_motor_velocities().tolist()
    self.motor_torques = self.get_motor_torques().tolist()
    self.base_orientation = list(self.get_base_orientation_euler())
    observation.extend(self.motor_angles)
    observation.extend(self.motor_velocities)
    observation.extend(self.motor_torques)
    observation.extend(self.base_orientation)
    return observation
  
  def norm_obs(self, obs):
    norm_obs = np.array([0]*self.obs_dim, dtype=np.float32)
    for i in range(0,12):
        norm_obs[i] = (obs[i]+MOTOR_ANGLE_MAX)/(MOTOR_ANGLE_MAX+MOTOR_ANGLE_MAX)
    for i in range(12,24):
        norm_obs[i] = np.clip((obs[i]+MOTOR_VELOCITY_MAX)/(MOTOR_VELOCITY_MAX+MOTOR_VELOCITY_MAX), 0, 1)
    for i in range(24, 36):
        norm_obs[i] = np.clip((obs[i]+MOTOR_TORQUE_MAX)/(MOTOR_TORQUE_MAX+MOTOR_TORQUE_MAX), 0, 1)
    norm_obs[36] = (obs[36]+BASE_ORIENTATION_MAX)/(BASE_ORIENTATION_MAX+BASE_ORIENTATION_MAX)
    norm_obs[37] = (obs[37]+BASE_ORIENTATION_MAX)/(BASE_ORIENTATION_MAX+BASE_ORIENTATION_MAX)
    norm_obs[38] = (obs[38]+BASE_ORIENTATION_MAX)/(BASE_ORIENTATION_MAX+BASE_ORIENTATION_MAX)

    return norm_obs
  
  def _denorm_obs(self, norm_obs):
    obs = np.array([0]*self.obs_dim, dtype=np.float32)
    for i in range(0,12):
        obs[i] = 2*norm_obs[i]*MOTOR_ANGLE_MAX - MOTOR_ANGLE_MAX
    for i in range(12,24):
        obs[i] = 2*norm_obs[i]*MOTOR_VELOCITY_MAX - MOTOR_VELOCITY_MAX
    for i in range(24, 36):
        obs[i] = 2*norm_obs[i]*MOTOR_TORQUE_MAX - MOTOR_TORQUE_MAX
    obs[36] = 2*BASE_ORIENTATION_MAX*norm_obs[36] - BASE_ORIENTATION_MAX
    obs[37] = 2*BASE_ORIENTATION_MAX*norm_obs[37] - BASE_ORIENTATION_MAX
    obs[38] = 2*BASE_ORIENTATION_MAX*norm_obs[38] - BASE_ORIENTATION_MAX

    return obs
  
  def set_kpkd(self, kp : float, kd : float):
    self._motor_kp = kp
    self._motor_kd = kd

  def set_motor_commands(self, angles, vels, torques):
    self.ref_angles = angles
    self.ref_vels = vels
    self.ref_torques = torques

  def _apply_action(self, motor_commands):
    """Set the desired motor commands to the motors of the mors.

    The desired motor commands are clipped based on the maximum allowed velocity.
    If the pd_control_enabled is True, a torque is calculated according to
    the difference between current and desired joint angle, as well as the joint
    velocity. This torque is exerted to the motor. For more information about
    PD control, please refer to: https://en.wikipedia.org/wiki/PID_controller.

    Args:
      motor_commands: The twelve desired motor angles or torques depending on torque_control_enabled variable.
    """
    current_motor_angle = self.get_motor_angles()
    if self._motor_velocity_limit < np.inf:
      
      motor_commands_max = (current_motor_angle + self._time_step * self._motor_velocity_limit)
      motor_commands_min = (current_motor_angle - self._time_step * self._motor_velocity_limit)
      motor_commands = np.clip(motor_commands, motor_commands_min, motor_commands_max)

    motor_commands_with_direction = np.multiply(motor_commands, self._motor_direction)
    self.motor_angles = current_motor_angle[:].tolist()  
    self.motor_velocities = self.get_motor_velocities().tolist() 

    if self._accurate_motor_model_enabled:
      self._accurate_motor_model.set_sensor_data(self.motor_angles, self.motor_velocities)

      if self._step_enabled == True:
        if self._torque_control_enabled:
          self._accurate_motor_model.set_ref_torque(motor_commands_with_direction)
          self._accurate_motor_model.set_kd(self._motor_kd)
        else:
          self._accurate_motor_model.set_ref_angle(motor_commands_with_direction)
          self._accurate_motor_model.set_kp(self._motor_kp)
          self._accurate_motor_model.set_kd(self._motor_kd)

      else:
        self._accurate_motor_model.set_ref_angle(self.ref_angles)
        self._accurate_motor_model.set_ref_vel(self.ref_vels)
        self._accurate_motor_model.set_ref_torque(self.ref_torques)
        self._accurate_motor_model.set_kp(self._motor_kp)
        self._accurate_motor_model.set_kd(self._motor_kd)

      
      self.tau = self._accurate_motor_model.step()

      for motor_id, tau in zip(self._motor_id_list, self.tau):
        self._SetMotorTorqueById(motor_id, float(tau))

    elif self._simple_motor_model_enabled:
        self._simple_motor_model.set_sensor_data(self.motor_angles, self.motor_velocities)

        if self._step_enabled == True:
          if self._torque_control_enabled:
            self._simple_motor_model.set_ref_torque(motor_commands_with_direction)
            self._simple_motor_model.set_kd(self._motor_kd)
          else:
            self._simple_motor_model.set_ref_angle(motor_commands_with_direction)
            self._simple_motor_model.set_kp(self._motor_kp)
            self._simple_motor_model.set_kd(self._motor_kd)

        else:
          self._simple_motor_model.set_ref_angle(self.ref_angles)
          self._simple_motor_model.set_ref_vel(self.ref_vels)
          self._simple_motor_model.set_ref_torque(self.ref_torques)
          self._simple_motor_model.set_kp(self._motor_kp)
          self._simple_motor_model.set_kd(self._motor_kd)
        
        self.tau = self._simple_motor_model.step()
        

        for motor_id, tau in zip(self._motor_id_list, self.tau):
          self._SetMotorTorqueById(motor_id, float(tau))
    else:

      if self._torque_control_enabled == False:
        if self._step_enabled == True:
          for motor_id, motor_command_with_direction in zip(self._motor_id_list, motor_commands_with_direction):
            self._SetMotorAngleById(motor_id, motor_command_with_direction)
        else:
          for motor_id, motor_command_with_direction in zip(self._motor_id_list, self.ref_angles):
            self._SetMotorAngleById(motor_id, motor_command_with_direction)

      else:
        if self._step_enabled == True:
          for motor_id, motor_command_with_direction in zip(self._motor_id_list, motor_commands_with_direction):
            self._SetMotorTorqueById(motor_id, float(motor_command_with_direction))
        else:
          for motor_id, motor_command_with_direction in zip(self._motor_id_list, self.ref_torques):
            self._SetMotorTorqueById(motor_id, float(motor_command_with_direction))

  def _RecordMassInfoFromURDF(self):
    self._base_mass_urdf = self._pybullet_client.getDynamicsInfo(self.quadruped, BASE_LINK_ID)[0]
    self._leg_masses_urdf = []
    self._leg_masses_urdf.append(
        self._pybullet_client.getDynamicsInfo(self.quadruped, LEG_LINK_ID[0])[0])
    self._leg_masses_urdf.append(
        self._pybullet_client.getDynamicsInfo(self.quadruped, MOTOR_LINK_ID[0])[0])

  def _BuildJointNameToIdDict(self):
    num_joints = self._pybullet_client.getNumJoints(self.quadruped)
    self._joint_name_to_id = {}
    for i in range(num_joints):
      joint_info = self._pybullet_client.getJointInfo(self.quadruped, i)
      self._joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]

  def _BuildMotorIdList(self):
    self._motor_id_list = [self._joint_name_to_id[motor_name] for motor_name in MOTOR_NAMES]
  
  def _SetMotorTorqueById(self, motor_id, torque : float):
    self._pybullet_client.setJointMotorControl2(bodyIndex=self.quadruped,
                                                jointIndex=motor_id,
                                                controlMode=pybullet.TORQUE_CONTROL,
                                                force=torque)

  def _SetMotorVelocityById(self, motor_id, desired_velocity):
    self._pybullet_client.setJointMotorControl2(bodyIndex=self.quadruped,
                                                jointIndex=motor_id,
                                                controlMode=self._pybullet_client.VELOCITY_CONTROL,
                                                targetVelocity=desired_velocity,
                                                #positionGain=self._kp,
                                                #velocityGain=self._kd,
                                                maxVelocity=self._motor_velocity_limit,
                                                force=self.force
                                                )
    
  def _SetMotorAngleById(self, motor_id, desired_angle):
    self._pybullet_client.setJointMotorControl2(bodyIndex=self.quadruped,
                                                jointIndex=motor_id,
                                                controlMode=self._pybullet_client.POSITION_CONTROL,
                                                targetPosition=desired_angle,
                                                #positionGain=self._kp,
                                                #velocityGain=self._kd,
                                                maxVelocity=self._motor_velocity_limit,
                                                force=self.force#self._max_force
                                                )

  def _SetMotorAngleByName(self, motor_name, desired_angle):
    self._SetMotorAngleById(self._joint_name_to_id[motor_name], desired_angle)

  def _ResetPose(self):
    """Reset the pose of the mors.
      constraint: Whether to add a constraint at the joints of two feet.
    """
    for i in range(self.num_legs):
      self._ResetPoseForLeg(i)

    for j in range(16):
            self._pybullet_client.setJointMotorControl2(self.quadruped, j, pybullet.VELOCITY_CONTROL, targetVelocity=0, force=0)

  def _ResetPoseForLeg(self, leg_id):
    """Reset the initial pose for the leg.

    Args:
      leg_id: It should be 0, 1, 2, or 3, which represents the leg at
        front_left, back_left, front_right and back_right.
    """
    knee_angle = 0
    shoulder_angle = 0
    hip_angle = 0

    leg_position = LEG_POSITION[leg_id]
    self._pybullet_client.resetJointState(self.quadruped,
                                          self._joint_name_to_id["abad_joint_" + leg_position],
                                          self._motor_direction[2 * leg_id] * shoulder_angle,
                                          targetVelocity=0)
    self._pybullet_client.resetJointState(self.quadruped,
                                          self._joint_name_to_id["hip_joint_" + leg_position],
                                          self._motor_direction[2 * leg_id] * knee_angle,
                                          targetVelocity=0)
    self._pybullet_client.resetJointState(self.quadruped,
                                          self._joint_name_to_id["knee_joint_" + leg_position],
                                          self._motor_direction[2 * leg_id + 1] * hip_angle,
                                          targetVelocity=0)


  def _reset_robot(self, reload_urdf=True):
    self._urdf_root = self._urdf_root.replace('.', '')
    addr = parentdir + self._urdf_root + "/mors.urdf"
    print(addr)
    if reload_urdf:
      if self._self_collision_enabled:
        self.quadruped = self._pybullet_client.loadURDF(
            addr,
            INIT_POSITION,
            flags=(pybullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT | pybullet.URDF_USE_SELF_COLLISION))
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 0, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 1, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 2, 1)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, 0, 1, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, 1, 2, 0)

        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 4, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 5, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 6, 1)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, 4, 5, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, 5, 6, 1)

        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 8, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 9, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 10, 1)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, 8, 9, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, 9, 10, 0)

        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 12, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 13, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, -1, 14, 1)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, 12, 13, 0)
        self._pybullet_client.setCollisionFilterPair(self.quadruped, self.quadruped, 13, 14, 0)
      else:
        self.quadruped = self._pybullet_client.loadURDF(
            addr, INIT_POSITION)
      if self._on_rack:
        self._pybullet_client.createConstraint(self.quadruped, BASE_LINK_ID, -1, -1,
                                               self._pybullet_client.JOINT_FIXED, [0, 0, 0],
                                               [0, 0, 0], INIT_POSITION)
      self._BuildJointNameToIdDict()
      self._BuildMotorIdList()
      self._RecordMassInfoFromURDF()
      self._ResetPose()

      for joint_number in range(self._pybullet_client.getNumJoints(self.quadruped)):
        self._pybullet_client.enableJointForceTorqueSensor(self.quadruped, joint_number)

    else:
      self._pybullet_client.resetBasePositionAndOrientation(self.quadruped, INIT_POSITION,
                                                            INIT_ORIENTATION)
      self._pybullet_client.resetBaseVelocity(self.quadruped, [0, 0, 0], [0, 0, 0])
      self._ResetPose()


    self._motor_enabled_list = [True] * self.num_motors
  
  def get_action_dimension(self):
    """Get the length of the action list.

    Returns:
      The length of the action list.
    """
    return self.action_dim

  def get_observation_upper_bound(self):
    """Get the upper bound of the observation.

    Returns:
    The upper bound of an observation. See GetObservation() for the details
        of each element of an observation.
    """
    upper_bound = np.array([0.0] * self.get_observation_dimension())
    upper_bound[:] = 1.0  # Quaternion of base orientation.
    return upper_bound

  def get_observation_lower_bound(self):
    """Get the lower bound of the observation."""
    lower_bound = np.array([0.0] * self.get_observation_dimension())
    lower_bound[:] = 0.0  # Quaternion of base orientation.
    return lower_bound

  def get_observation_dimension(self):
    """Get the length of the observation list.

    Returns:
    The length of the observation list.
    """
    return self.obs.size
  
  #--------------------------
  # get sensor data
  #--------------------------
  def get_base_orientation(self):
    """Get the orientation of mors's base, represented as quaternion.

    Returns:
      The orientation of mors's base.
    """
    _, orientation = (self._pybullet_client.getBasePositionAndOrientation(self.quadruped))
    return orientation

  def get_base_orientation_euler(self):
    """Get the orientation of mors's base, represented as Euler angles.

    Returns:
      The orientation of mors's base.
    """
    _, orientation = (self._pybullet_client.getBasePositionAndOrientation(self.quadruped))
    euler_orient = self._pybullet_client.getEulerFromQuaternion(orientation)
    return euler_orient

  def get_base_position_and_orientation(self):
    position, quat = self._pybullet_client.getBasePositionAndOrientation(self.quadruped)
    return position, quat

  def get_base_lin_vel(self):
    vel = self._pybullet_client.getBaseVelocity(self.quadruped)
    return vel[0]

  def get_base_ang_vel(self):
    vel = self._pybullet_client.getBaseVelocity(self.quadruped)
    return vel[1]
  
  def get_motor_states(self):
    joint_number_range = range(self._pybullet_client.getNumJoints(self.quadruped))
    joint_states = self._pybullet_client.getJointStates(self.quadruped, joint_number_range)
    joint_infos = [self._pybullet_client.getJointInfo(self.quadruped, i) for i in joint_number_range]
    joint_states, joint_name = \
        zip(*[(j, i[1]) for j, i in zip(joint_states, joint_infos) if i[2] != self._pybullet_client.JOINT_FIXED])
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_reaction_forces = [state[2] for state in joint_states]
    joint_torques = [0]*12
    for i in range(12):
      joint_torques[i] = joint_reaction_forces[i][5]
    # print(f"{[ '%.3f' % elem for elem in joint_torques ]}")
    # print(joint_name)
    return joint_positions, joint_velocities, joint_torques, joint_name
  
  def get_motor_angles(self):
    """Get the eight motor angles at the current moment.

    Returns:
      Motor angles.
    """
    motor_angles = [
        self._pybullet_client.getJointState(self.quadruped, motor_id)[0]
        for motor_id in self._motor_id_list
    ]
    motor_angles = np.multiply(motor_angles, self._motor_direction)
    return motor_angles

  def get_motor_velocities(self):
    """Get the velocity of all motors.

    Returns:
      Velocities of all motors.
    """
    motor_velocities = [
        self._pybullet_client.getJointState(self.quadruped, motor_id)[1]
        for motor_id in self._motor_id_list
    ]
    motor_velocities = np.multiply(motor_velocities, self._motor_direction)
    return motor_velocities

  def get_motor_torques(self):
    """Get the amount of torques the motors are exerting.

    Returns:
      Motor torques of all twelve motors.
    """
    motor_torques = [
        self._pybullet_client.getJointState(self.quadruped, motor_id)[3]
        for motor_id in self._motor_id_list
    ]
    motor_torques = np.multiply(motor_torques, self._motor_direction)
    return motor_torques

  def GetCOMPos(self):
    base_pos, base_orientation = self._pybullet_client.getBasePositionAndOrientation(self.quadruped)
    com_pos_loc = [-0.002185, -0.000018, 0.000286]
    com_orient_loc = [0.0, 0.0, 0.0, 1.0]
    
    com_pos, com_orient = self._pybullet_client.multiplyTransforms(com_pos_loc, com_orient_loc, base_pos, base_orientation)
    return com_pos

  def GetEFPos(self):
    r1_pos, r1_or,_,_,_,_  = self._pybullet_client.getLinkState(self.quadruped, 4)
    l1_pos, l1_or,_,_,_,_  = self._pybullet_client.getLinkState(self.quadruped, 12)
    r2_pos, r2_or,_,_,_,_  = self._pybullet_client.getLinkState(self.quadruped, 8)
    l2_pos, l2_or,_,_,_,_  = self._pybullet_client.getLinkState(self.quadruped, 16)

    ef = [0]*12
    for i in range(3):
      ef[i] = r1_pos[i]
      ef[i+3] = l1_pos[i]
      ef[i+6] = r2_pos[i]
      ef[i+9] = l2_pos[i]

    return ef
  
  def get_contact_flags(self):
    self.contact_points = self._pybullet_client.getContactPoints(self.quadruped)
    contact_flags = [False]*4
    
    for contact_point in self.contact_points:
      if contact_point[3] == 3:
          contact_flags[1] = True
      if contact_point[3] == 7:
          contact_flags[0] = True
      if contact_point[3] == 11:
          contact_flags[3] = True
      if contact_point[3] == 15:
          contact_flags[2] = True
    return self.contact_points, contact_flags


  # ----------------------
  # Physical Parameters
  # ----------------------
  def get_base_mass_from_urdf(self):
    """Get the mass of the base from the URDF file."""
    return self._base_mass_urdf

  def get_leg_masses_from_urdf(self):
    """Get the mass of the legs from the URDF file."""
    return self._leg_masses_urdf

  def set_base_mass(self, base_mass):
    self._pybullet_client.changeDynamics(self.quadruped, BASE_LINK_ID, mass=base_mass)

  def set_leg_masses(self, shoulder, hip, knee):
    for i in range(4):
      self._pybullet_client.changeDynamics(self.quadruped, i*4, mass=shoulder)
      self._pybullet_client.changeDynamics(self.quadruped, i*4+1, mass=hip)
      self._pybullet_client.changeDynamics(self.quadruped, i*4+2, mass=knee)

  def set_foot_friction(self, foot_friction):
    """Set the lateral friction of the feet.

    Args:
      foot_friction: The lateral friction coefficient of the foot. This value is
        shared by all four feet.
    """
    # for link_id in FOOT_LINK_ID:
      # self._pybullet_client.changeDynamics(self.quadruped, link_id, lateralFriction=foot_friction)
    self._pybullet_client.changeDynamics(self.quadruped, 3, lateralFriction=self._lateral_friction, spinningFriction=self._spinning_friction)
    self._pybullet_client.changeDynamics(self.quadruped, 7, lateralFriction=self._lateral_friction, spinningFriction=self._spinning_friction)
    self._pybullet_client.changeDynamics(self.quadruped, 11, lateralFriction=self._lateral_friction, spinningFriction=self._spinning_friction)
    self._pybullet_client.changeDynamics(self.quadruped, 15, lateralFriction=self._lateral_friction, spinningFriction=self._spinning_friction)

  def set_friction(self, lateral_friction=1.0, spinning_friction=0.0063):
    self._lateral_friction = lateral_friction
    self._spinning_friction = spinning_friction

  def set_joint_forces(self, force):
    self.force = force

  def set_joint_max_velocity(self, velocity):
    self._motor_velocity_limit = velocity

  def set_control_mode(self, mode):
    self._control_mode = mode
    if mode == pybullet.TORQUE_CONTROL:
      if self.first == True:
        self.first = False
        for j in range(16):
              self._pybullet_client.setJointMotorControl2(self.quadruped, j, pybullet.VELOCITY_CONTROL, force=0)
    else:
      self.first = True

  # ----------------------
  # Terrain Parameters
  # ----------------------
  def set_world(self, world):
    self._world_name = world

  # def set_reset_type(self, type : bool):
  #   self._hard_reset = type

  # ----------------------
  # External Forces
  # ----------------------
  def set_ext_forces_params(self, magn, duration, interval): #duration and interval in timesteps
    self._ext_force_magn = magn
    self._ext_force_duration = duration
    self._ext_force_interval = interval

  def _apply_force(self):
    if 100 < (self._env_step_counter%self._ext_force_interval) <= (100+self._ext_force_duration):
        if (self._env_step_counter%self._ext_force_interval) < 102:
            self._force_dir = -self._force_dir
        force = [0, self._force_dir*self._ext_force_magn, 0]
        print(force)
        self._pybullet_client.applyExternalForce(self.quadruped, -1, force, [0,0,0], pybullet.LINK_FRAME)
        com_pos = self.GetCOMPos()
        self._pybullet_client.addUserDebugLine([com_pos[0], com_pos[1], com_pos[2]], 
                                                    [com_pos[0]+force[0]/1000, com_pos[1]+force[1]/1000, com_pos[2]+force[2]/1000], 
                                                    [1, 0, 0], 3, 0.4)
        
