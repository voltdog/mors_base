import numpy as np

from zmp_controller.BodyMovingControl import BodyMovingControl
from zmp_controller.FootPlanner import FootPlanner
from zmp_controller.SwingTrajectoryGenerator import SwingTrajectoryGenerator
from zmp_controller.ZMPTrajectoryGenerator import ZMPTrajectoryGenerator
from zmp_controller.ZMP_Preview_Control import ZMP_Preview_Control
from zmp_controller.ikine import IKineQuadruped
from zmp_controller.fkine import FKineQuadruped

R1 = 0
L1 = 1
R2 = 2
L2 = 3

X = 0
Y = 1
Z = 2

class ZMPLocomotion():
    def __init__(self, 
                 FPS=240, 
                 real_robot=False, 
                 t_preview=1.6, 
                 stride_frequency=1.5,
                 cog_offset_x=0.0,
                 cog_offset_y=0.0,
                 cog_offset_z=0.0,
                 ef_init_x=0.143,
                 ef_init_y=0.13,
                 robot_height=0.17,
                 kinematic_scheme='x',
                 ):
        
        self.FPS = FPS
        self.real_robot = real_robot
        self.t_preview = t_preview
        self.pre_t_preview = t_preview
        self.stride_frequency = stride_frequency
        self.cog_offset_x = cog_offset_x
        self.cog_offset_y = cog_offset_y
        self.cog_offset_z = cog_offset_z
        self.ef_init_x = ef_init_x
        self.ef_init_y = ef_init_y
        self.robot_height = robot_height
        self.kinematic_scheme = kinematic_scheme
        self.cnt = 1

        self.Nl = int(t_preview * FPS) # количество временных шагов для предсказания
        
        com_real = [self.cog_offset_x, self.cog_offset_y, self.robot_height]

            # com_real = [-0.002185, -0.000018, 0.000286]

        self.__P_B3 = [[self.ef_init_x, -self.ef_init_y, -self.robot_height],
                [self.ef_init_x, self.ef_init_y, -self.robot_height],
                [-self.ef_init_x, -self.ef_init_y, -self.robot_height],
                [-self.ef_init_x, self.ef_init_y, -self.robot_height]]

        t_stride = 1/(2*stride_frequency) # время на один шаг

        self.foot_planner = FootPlanner(ef_init_x=self.ef_init_x, 
                                        ef_init_y=self.ef_init_y)
        self.zmp_traj_generator = ZMPTrajectoryGenerator(com_real=com_real)
        self.zmp_ctl = ZMP_Preview_Control(sim_freq=FPS)
        self.zmp_ctl.init_regulator(z_c = self.robot_height+self.cog_offset_z,
                        t_step=t_stride,
                        t_preview=t_preview,
                        Q=0.5,
                        R=1e-4)

        self.body_moving = BodyMovingControl()
        self.swing_traj = SwingTrajectoryGenerator(sim_freq=FPS,
                                                   ef_init_x=self.ef_init_x,
                                                   ef_init_y=self.ef_init_y,
                                                   robot_height=self.robot_height)
        self.fk = FKineQuadruped(theta_offset=[0, -np.pi/2, 0])
        self.ik = IKineQuadruped(theta_offset=[0, -np.pi/2, 0])

        self.e_r = 0
        self.d_e_r = 0
        self.e_r_prev = 0
        self.i_e_r = 0

        self.e_p = 0
        self.d_e_p = 0
        self.e_p_prev = 0
        self.i_e_p = 0

        self.pitch_cur = 0
        self.roll_cur = 0
        self.yaw = 0
        self.body_lin_vel = 0

        self.v_err = [0,0]

    def set_walking_params(self, v_x, v_y, stride_frequency, stride_height, rot_dir, rot_r):
        self.v_x = v_x
        self.v_y = v_y
        self.stride_frequency = stride_frequency
        self.Sh = stride_height
        self.rot_dir = rot_dir
        self.rot_r = rot_r

    def set_feedback(self, euler, lin_vel):
        self.roll_cur = euler[0]
        self.pitch_cur = euler[1]
        self.yaw = euler[2]
        self.body_lin_vel = lin_vel[:]

    def get_state(self):
        return self.cur_active_legs

    def step(self, it):
        # print("==================")
        # Prepare data
        # if np.abs(self.v_x) < 2.4:
        #     self.t_preview = 0.4
        # else:
        #     self.t_preview = 2.6

        if self.stride_frequency != 0:
            t_stride = 1/(2*self.stride_frequency) # время на один шаг
        else:
            t_stride = 1000000000
            
        # if self.pre_t_preview != self.t_preview:
        #     self.zmp_ctl.init_regulator(z_c = self.robot_height+self.cog_offset_z,
        #                 t_step=t_stride,
        #                 t_preview=self.t_preview,
        #                 Q=0.5,
        #                 R=1e-4)
        #     print(f"t_preview: {self.t_preview}")

        self.pre_t_preview = self.t_preview

        
        cnt_stride = int(t_stride*self.FPS) # количество временных шагов для одного шага робота
        self.Nl = int(self.t_preview * self.FPS) # количество временных шагов для предсказания
        preview_strides = int(np.ceil(self.t_preview/t_stride)) # сколько шагов надо предсказать
        Sl_x = t_stride * self.v_x # длина шага по x
        Sl_y = t_stride * self.v_y # длина шага по y
        # print(f"Sl_x {Sl_x}")
 
        self.zmp_ctl.set_gait_params(t_step=t_stride)
        self.swing_traj.set_gait_params(self.stride_frequency, cnt_stride)

        # ----------------------------------------------------------------
        # предскажем координаты будущих шагов
        # Foot Planner
        # self.v_err[0] = self.body_lin_vel[0] - self.v_x
        # self.v_err[1] = self.body_lin_vel[1] - self.v_y
        self.foot_planner.set_params(preview_strides=preview_strides,
                                Sl_x=Sl_x,
                                Sl_y=Sl_y)#, v_err=self.v_err)
        leg_lst, self.cur_active_legs = self.foot_planner.step(self.cnt)
        # print(f"{it}, {Sl_x}, {leg_lst[1][0][0]}, {leg_lst[1][0][1]}, {leg_lst[1][1][0]}, {leg_lst[1][1][1]}")
        # ----------------------------------------------------------------

        # ----------------------------------------------------------------
        # ZMP Trajectory Generator
        self.zmp_traj_generator.set_params(cnt_stride=cnt_stride, Nl=self.Nl)
        zmp_traj_x, zmp_traj_y = self.zmp_traj_generator.step(it=it, cnt=self.cnt, leg_lst=leg_lst)
        # ----------------------------------------------------------------

        # ----------------------------------------------------------------
        # Preview Controller
        ZMP_x_preview = np.asmatrix(zmp_traj_x).T
        ZMP_y_preview = np.asmatrix(zmp_traj_y).T
        self.zmp_ctl.set_zmp_trajectory(ZMP_x_preview, ZMP_y_preview)
        com_ref, zmp_pos = self.zmp_ctl.step()
        # ----------------------------------------------------------------

        # ----------------------------------------------------------------
        # Transfer from global to body reference 
        p_cur_loc = [0]*12
        p_cur_glob = [[leg_lst[0][0][X], leg_lst[0][0][Y], -self.robot_height], 
                    [leg_lst[0][1][X], leg_lst[0][1][Y], -self.robot_height], 
                    [leg_lst[0][2][X], leg_lst[0][2][Y], -self.robot_height], 
                    [leg_lst[0][3][X], leg_lst[0][3][Y], -self.robot_height]] 
        x_b, y_b, z_b, roll_b, pitch_b, yaw_b = [0, 0, 0, 0, 0, 0]
        for i in range(4):
            p_cur_loc[3*i] = p_cur_glob[i][X]
            p_cur_loc[3*i+1] = p_cur_glob[i][Y]
            p_cur_loc[3*i+2] = p_cur_glob[i][Z]
        # ----------------------------------------------------------------

        # ----------------------------------------------------------------
        # Body Moving
        com_offset = [com_ref[0]-x_b, com_ref[1]-y_b, 0]
        # if cnt == 1:
        #     ang_offset[2] = 0
        self.body_moving.set_body_ang_pos([0,0,0])
        self.body_moving.set_body_lin_pos(com_offset)
        p_ref_move_loc = self.body_moving.step(p_cur_loc)
        # ----------------------------------------------------------------
        
        # ----------------------------------------------------------------
        # Swing Trajectory Generator
        p_start = []
        p_rise = []
        p_finish = []
        for i in range(4):
            capture_point_x = 0#-0.13*(self.body_lin_vel[0] - self.v_x)
            capture_point_y = 0#-0.13*(self.body_lin_vel[1] - self.v_y)
            p_start.append([leg_lst[0][i][X], leg_lst[0][i][Y], -self.robot_height]) # в будущем поменять. Точкой должна быть та, на которой закончилась фаза опоры
            p_finish.append([leg_lst[1][i][X]+capture_point_x, leg_lst[1][i][Y]+capture_point_y, -self.robot_height])
            p_rise.append([(p_start[i][X] + (p_finish[i][X]-p_start[i][X])/2)-com_offset[X]-Sl_x/4, 
                            (p_start[i][Y] + (p_finish[i][Y]-p_start[i][Y])/2)-com_offset[Y]-Sl_x/4, 
                            -self.robot_height+self.Sh])

        self.swing_traj.set_points(p_start, p_rise, p_finish)
        p_ref_swing_glob = self.swing_traj.step(it=it, cnt=self.cnt-1, active_legs=self.cur_active_legs)
        # print(f"{it}, {self.cnt}, {p_rise[0][0]}, {p_rise[0][1]}, {p_start[0][0]}, {p_start[0][1]}, {p_rise[1][0]}, {p_rise[1][1]}, {p_start[1][0]}, {p_start[1][1]}")
        # print(f"{it}, {p_ref_swing_glob[0]}, {p_ref_swing_glob[1]}, {p_ref_swing_glob[2]}, {p_ref_swing_glob[3]}, {p_ref_swing_glob[4]}, {p_ref_swing_glob[5]}")
        # ----------------------------------------------------------------

        # ----------------------------------------------------------------
        # MUX
        # Transfer from global to local coordinates
        p_ref_swing_loc = [0]*12
        for i in range(4):
            p_ref_swing_loc[3*i] = p_ref_swing_glob[3*i] - com_offset[X] - Sl_x/4
            p_ref_swing_loc[3*i+1] = p_ref_swing_glob[3*i+1] - com_offset[Y] - Sl_y/4
            p_ref_swing_loc[3*i+2] = p_ref_swing_glob[3*i+2]
            p_ref_move_loc[3*i] -= ((Sl_x/4))
            p_ref_move_loc[3*i+1] -= ((Sl_y/4))

        # print(f"{it}, {p_ref_swing_loc[0]}, {p_ref_swing_loc[1]}, {p_ref_swing_loc[2]}, {p_ref_swing_loc[3]}, {p_ref_swing_loc[4]}, {p_ref_swing_loc[5]}, \
        #       {com_ref[X]}, {com_ref[Y]}, {Sl_x}, {Sl_y}")
    
        # Mux
        p_ref_loc = [0]*12
        if self.cur_active_legs[L1] and self.cur_active_legs[R2]:
            p_ref_loc[0:3] = p_ref_move_loc[0:3]
            p_ref_loc[3:6] = p_ref_swing_loc[3:6]
            p_ref_loc[6:9] = p_ref_swing_loc[6:9]
            p_ref_loc[9:12] = p_ref_move_loc[9:12]
        elif self.cur_active_legs[R1] and self.cur_active_legs[L2]:
            p_ref_loc[0:3] = p_ref_swing_loc[0:3]
            p_ref_loc[3:6] = p_ref_move_loc[3:6]
            p_ref_loc[6:9] = p_ref_move_loc[6:9]
            p_ref_loc[9:12] = p_ref_swing_loc[9:12]
        # ----------------------------------------------------------------

        # ----------------------------------------------------------------
        # Turning
        if self.rot_dir != 0:
            for i in range(4):
                p_ref_loc[3*i+0] -= self.__P_B3[i][0]
                p_ref_loc[3*i+1] -= self.__P_B3[i][1]
                if self.rot_dir == -1:
                    alpha = -np.arctan2(2*self.ef_init_x, 2*self.rot_r-2*self.ef_init_y)
                    beta = -np.arctan2(2*self.ef_init_x, 2*self.rot_r+2*self.ef_init_y)
                else:
                    alpha = np.arctan2(2*self.ef_init_x, 2*self.rot_r+2*self.ef_init_y)
                    beta = np.arctan2(2*self.ef_init_x, 2*self.rot_r-2*self.ef_init_y)

                if i == 0 or i == 1:
                    sign = 1
                else:
                    sign = -1

                if i == 0 or i == 2:
                    turn = alpha
                else:
                    turn = beta
                    
                p_ref_loc[3*i+0] = p_ref_loc[3*i+0]*np.cos(sign*turn) - p_ref_loc[3*i+1]*np.sin(sign*turn)
                p_ref_loc[3*i+1] = p_ref_loc[3*i+0]*np.sin(sign*turn) + p_ref_loc[3*i+1]*np.cos(sign*turn)
                p_ref_loc[3*i+0] += self.__P_B3[i][0]
                p_ref_loc[3*i+1] += self.__P_B3[i][1]
        # ----------------------------------------------------------------
        # Active Balance
        # get pitch and roll angles
        # roll_cur, pitch_cur = self.roll_cur, self.pitch_cur
        # set zero refrence angles
        roll_ref, pitch_ref = 0, 0.0
        # find u_r using PID
        kp = 0
        kd = 0
        ki = 0
        kp_r = 0.0#0.3#2#4
        kd_r = 0.0
        kp_p = 0.3#2
        kd_p = 0.0
        # ki = 0.001

        self.e_r = roll_ref - self.roll_cur
        self.d_e_r = self.e_r - self.e_r_prev
        self.i_e_r += self.e_r
        self.u_r = kp_r*self.e_r + kd_r*self.d_e_r + ki*self.i_e_r
        # find u_p using PID
        self.e_p = pitch_ref - self.pitch_cur
        self.d_e_p = self.e_p - self.e_p_prev
        self.i_e_p += self.e_p
        self.u_p = kp_p*self.e_p + kd_p*self.d_e_p + ki*self.i_e_p
        # utilize body_moving in order to change body orientation
        self.body_moving.set_body_ang_pos_incr([-self.u_r, -self.u_p, 0])
        self.body_moving.set_body_lin_pos([0, 0, 0])
        p_ref_final = self.body_moving.step(p_ref_loc)
        # save previous results
        self.e_r_prev = self.e_r
        self.e_p_prev = self.e_p
        # ----------------------------------------------------------------
        # Inverse Kinematics 
        # print(f"{it}, {p_ref_final[0]}, {p_ref_final[1]}, {p_ref_final[2]}, {p_ref_final[3]}, {p_ref_final[4]}, {p_ref_final[5]}")
        theta_ref = self.ik.calculate(p_ref_final, config=self.kinematic_scheme)

        # Counter
        if self.cnt >= cnt_stride:
            # stride_quantity += 1
            self.cnt = 1
        else:
            self.cnt += 1

        return theta_ref