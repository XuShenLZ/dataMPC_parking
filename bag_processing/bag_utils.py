#!/usr/bin python3

import rosbag
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import warnings
import yaml, os

# In order for this import statement to work we need to build the python package
# 1. Navigate to mpclab_strategy_obca/src/mpclab_strategy_obca
# 2. run 'python -m pip install .'
from mpclab_strategy_obca.constraint_generation.hyperplaneConstraintGenerator import hyperplaneConstraintGenerator
from mpclab_strategy_obca.utils.types import experimentParams

class Vehicle(object):
    """
    Vehicle Class as EV / TV data structure
    """
    def __init__(self):
        """
        Initializing
        """
        self.state_t0 = None
        self.state_t = []

        self.x = []
        self.y = []
        self.heading = []
        self.v = []

        self.input_t0 = None
        self.input_t = []
        self.delta = []
        self.a = []

        self.pred = None
    
    def append_state(self, msg, t):
        """
        append vehicle state
        """
        if self.state_t0 == None:
            self.state_t0 = t
        
        self.state_t.append(t)

        self.x.append(msg.x)
        self.y.append(msg.y)
        self.heading.append(msg.psi)
        self.v.append(msg.v)

    def append_input(self, msg, t):
        """
        append vehicle input
        """
        if self.input_t0 == None:
            self.input_t0 = t
        
        self.input_t.append(t)

        self.delta.append(msg.servo)
        self.a.append(msg.motor)

    def append_pred_state_input(self, msg):
        """
        append predicted vehicle state and input
        """
        if self.pred == None:
            self.pred = Vehicle()
        
        self.pred.x.append(msg.x)
        self.pred.y.append(msg.y)
        self.pred.heading.append(msg.psi)
        self.pred.v.append(msg.v)

        self.pred.delta.append(msg.df)
        self.pred.a.append(msg.a)

    def slicing_data(self, start_time, end_time):
        """
        take a slice of the time and states
        """
        # For State
        start_time_diff = np.abs(np.array(self.state_t) - start_time)
        state_start_idx = np.argmin(start_time_diff)

        end_time_diff = np.abs(np.array(self.state_t) - end_time)
        state_end_idx = np.argmin(end_time_diff)

        # self.state_t0 = self.state_t[state_start_idx]
        # self.state_t = [
        #     x-self.state_t0 for x in self.state_t[state_start_idx: state_end_idx]]
        self.state_t = self.state_t[state_start_idx: state_end_idx]
        self.state_t0 = self.state_t[0]

        self.x = self.x[state_start_idx: state_end_idx]
        self.y = self.y[state_start_idx: state_end_idx]
        self.heading = self.heading[state_start_idx: state_end_idx]
        self.v = self.v[state_start_idx: state_end_idx]

        # For input
        start_time_diff = np.abs(np.array(self.input_t) - start_time)
        input_start_idx = np.argmin(start_time_diff)

        end_time_diff = np.abs(np.array(self.input_t) - end_time)
        input_end_idx = np.argmin(end_time_diff)

        # self.input_t0 = self.input_t[input_start_idx]
        # self.input_t = [
        #     x-self.input_t0 for x in self.input_t[input_start_idx: input_end_idx]]
        self.input_t = self.input_t[input_start_idx: input_end_idx]
        self.input_t0 = self.input_t[0]

        self.delta = self.delta[input_start_idx: input_end_idx]
        self.a = self.a[input_start_idx: input_end_idx]

class FiniteStateMachine(object):
    """
    Finite State Machine Data Structure
    """
    def __init__(self):
        """
        Initialization
        """
        self.t0 = None
        self.t = []

        self.states = []
        self.state_idxs = []
        self.state_names = ["*-OBCA", "Safety", "EB"]

        self.score_l = []
        self.score_r = []
        self.score_y = []

    def append_time(self, t):
        """
        append time
        """
        if self.t0 == None:
            self.t0 = t
        
        self.t.append(t)

    def append_state(self, state):
        """
        append state and convert it into index
        """
        self.states.append(state)

        if state in ["Free-Driving", "HOBCA-Unlocked", "HOBCA-Locked"]:
            idx = 0
        elif state in ["Safe-Confidence", "Safe-Yield", "Safe-Infeasible"]:
            idx = 1
        elif state == "Emergency-Break":
            idx = 2
        else:
            raise ValueError("State not recognized.")

        self.state_idxs.append(idx)

    def append_score(self, scores):
        """
        append score to each strategy
        """
        self.score_l.append(scores[0])
        self.score_r.append(scores[1])
        self.score_y.append(scores[2])
    def slicing_data(self, start_time, end_time):
        """
        slicing data according to start and end time
        """
        start_time_diff = np.abs(np.array(self.t) - start_time)
        start_idx = np.argmin(start_time_diff)

        end_time_diff = np.abs(np.array(self.t) - end_time)
        end_idx = np.argmin(end_time_diff)

        # self.t0 = self.t[start_idx]
        # self.t = [x-self.t0 for x in self.t[start_idx: end_idx]]
        self.t = self.t[start_idx:end_idx]
        self.t0 = self.t[0]

        self.states = self.states[start_idx:end_idx]
        self.state_idxs = self.state_idxs[start_idx:end_idx]

        self.score_l = self.score_l[start_idx:end_idx]
        self.score_r = self.score_r[start_idx:end_idx]
        self.score_y = self.score_y[start_idx:end_idx]

# Object which reads the rosbag. Use method get_frame to grab a frame of the video and the data up to that point in time
class VideoDataReader(object):
    def __init__(self, filename, params_dir):
        bridge = CvBridge()
        
        image_topic='/overhead_camera/image_rect_color'
        fsm_topic = '/ego_vehicle/fsm_state'
        score_topic = '/ego_vehicle/strategy_scores'
        ev_state_topic = '/ego_vehicle/est_states'
        ev_input_topic = '/ego_vehicle/ecu'
        ev_pred_topic = '/ego_vehicle/pred_states'
        tv_state_topic = '/target_vehicle/est_states'
        tv_input_topic = '/target_vehicle/ecu'
        tv_pred_topic = '/target_vehicle/pred_states'
        
        ev_control_params_file = os.path.join(params_dir, 'params', 'ego_vehicle', 'controller.yaml')
        with open(ev_control_params_file, 'r') as f:
            self.ev_control_params = yaml.safe_load(f)
        ev_vehicle_params_file = os.path.join(params_dir, 'params', 'ego_vehicle', 'vehicle.yaml')
        with open(ev_vehicle_params_file, 'r') as f:
            self.ev_vehicle_params = yaml.safe_load(f)
        tv_control_params_file = os.path.join(params_dir, 'params', 'target_vehicle', 'controller.yaml')
        with open(tv_control_params_file, 'r') as f:
            self.tv_control_params = yaml.safe_load(f)
        tv_vehicle_params_file = os.path.join(params_dir, 'params', 'target_vehicle', 'vehicle.yaml')
        with open(tv_vehicle_params_file, 'r') as f:
            self.tv_vehicle_params = yaml.safe_load(f)
        
        self.dt = self.ev_control_params['controller']['dt']
        self.N = self.ev_control_params['controller']['obca']['N']
        self.v_ref = self.ev_control_params['controller']['obca']['v_ref']
        EV_L = self.ev_vehicle_params['car']['plot']['L']
        EV_W = self.ev_vehicle_params['car']['plot']['W']
        TV_L = self.tv_vehicle_params['car']['plot']['L']
        TV_W = self.tv_vehicle_params['car']['plot']['W']
        coll_buf_r = np.sqrt(EV_L**2+EV_W**2)/2
        params = experimentParams(car_L=TV_L, car_W=TV_W, collision_buffer_r=coll_buf_r)
        self.constraint_generator = hyperplaneConstraintGenerator(params)
        
        self.image_t = []
        self.image = []
        self.fsm_t = []
        self.fsm = []
        self.score_t = []
        self.score = []
        self.ev_state_t = []
        self.ev_state = []
        self.ev_input_t = []
        self.ev_input = []
        self.ev_pred_t = []
        self.ev_pred = []
        self.tv_state_t = []
        self.tv_state = []
        self.tv_input_t = []
        self.tv_input = []
        self.tv_pred_t = []
        self.tv_pred = []
        
        with rosbag.Bag(filename, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == image_topic:
                    cv_image = bridge.imgmsg_to_cv2(msg, 'rgb8')
                    self.image_t.append(t.to_sec())
                    self.image.append(cv_image)
                elif topic == fsm_topic:
                    self.fsm_t.append(t.to_sec())
                    self.fsm.append(msg.fsm_state)
                elif topic == score_topic:
                    self.score_t.append(t.to_sec())
                    self.score.append(msg.scores)
                elif topic == ev_state_topic:
                    z = np.array([msg.x, msg.y, msg.psi, msg.v])
                    self.ev_state_t.append(t.to_sec())
                    self.ev_state.append(z)
                elif topic == ev_input_topic:
                    u = np.array([msg.servo, msg.motor])
                    self.ev_input_t.append(t.to_sec())
                    self.ev_input.append(u)
                elif topic == ev_pred_topic:
                    pred = np.vstack((msg.x, msg.y, msg.psi, msg.v)).T
                    self.ev_pred_t.append(t.to_sec())
                    self.ev_pred.append(pred)
                elif topic == tv_state_topic:
                    z = np.array([msg.x, msg.y, msg.psi, msg.v])
                    self.tv_state_t.append(t.to_sec())
                    self.tv_state.append(z)
                elif topic == tv_input_topic:
                    u = np.array([msg.servo, msg.motor])
                    self.tv_input_t.append(t.to_sec())
                    self.tv_input.append(u)
                elif topic == tv_pred_topic:
                    pred = np.vstack((msg.x, msg.y, msg.psi, msg.v)).T
                    self.tv_pred_t.append(t.to_sec())
                    self.tv_pred.append(pred)
        
        self.image_t = np.array(self.image_t)
        self.video_start = np.amin(self.image_t)
        self.video_end = np.amax(self.image_t)
        self.video_length = self.video_end - self.video_start
        self.n_frames = len(self.image_t)
        
        print('Video of duration %g s read in as %i frames' % (self.video_length, self.n_frames))
    
    def get_frame(self, time=None, frame_idx=None):
        if time is None and frame_idx is None:
            raise(RuntimeError('Need to specify a time between up to %g s or a frame number between up to %i' % (self.video_length, self.n_frames-1)))
        if time is not None and frame_idx is not None:
            raise(RuntimeError('Can only specify either time or frame'))
            
        if time is not None:
            assert time >= 0, 'Time must be non-negative'
            if time > self.video_length:
                warnings.warn('Desired time of %g s is greater than video length of %g s, returning last frame' % (time, self.video_length))
                time = self.video_length
            frame_idx = np.argmin(np.abs(self.image_t - (time+self.video_start)))
        elif frame_idx is not None:
            assert frame_idx >= 0, 'Frame number must non-negative'
            if frame_idx >= self.n_frames:
                warnings.warn('Desired frame number of %i is greater than %i, returning last frame' % (frame_idx, self.n_frames))
                frame_idx = self.n_frames[-1]
            time = self.image_t[frame_idx] - self.video_start
        
        data = dict()
        
        t_idx = np.argmin(np.abs(self.fsm_t - (time+self.video_start)))+1
        data['fsm_t'] = np.array(self.fsm_t[:t_idx])
        data['fsm'] = np.array(self.fsm[:t_idx])
        
        t_idx = np.argmin(np.abs(self.score_t - (time+self.video_start)))+1
        data['score_t'] = np.array(self.score_t[:t_idx])
        data['score'] = np.array(self.score[:t_idx])
        
        t_idx = np.argmin(np.abs(self.ev_state_t - (time+self.video_start)))+1
        data['ev_state_t'] = np.array(self.ev_state_t[:t_idx])
        data['ev_state'] = np.array(self.ev_state[:t_idx])
        
        t_idx = np.argmin(np.abs(self.ev_input_t - (time+self.video_start)))+1
        data['ev_input_t'] = np.array(self.ev_input_t[:t_idx])
        data['ev_input'] = np.array(self.ev_input[:t_idx])
        
        t_idx = np.argmin(np.abs(self.ev_pred_t - (time+self.video_start)))+1
        data['ev_pred_t'] = np.array(self.ev_pred_t[t_idx])
        data['ev_pred'] = np.array(self.ev_pred[t_idx])
        
        t_idx = np.argmin(np.abs(self.tv_state_t - (time+self.video_start)))+1
        data['tv_state_t'] = np.array(self.tv_state_t[:t_idx])
        data['tv_state'] = np.array(self.tv_state[:t_idx])
        
        t_idx = np.argmin(np.abs(self.tv_input_t - (time+self.video_start)))+1
        data['tv_input_t'] = np.array(self.tv_input_t[:t_idx])
        data['tv_input'] = np.array(self.tv_input[:t_idx])
        
        t_idx = np.argmin(np.abs(self.tv_pred_t - (time+self.video_start)))+1
        data['tv_pred_t'] = np.array(self.tv_pred_t[t_idx])
        data['tv_pred'] = np.array(self.tv_pred[t_idx])
        
        EV_x, EV_y, EV_heading, EV_v = data['ev_state'][-1]
        TV_x, TV_y, TV_heading, TV_v = data['tv_state'][-1]
        TV_pred = data['tv_pred']
        
        X_ref = EV_x + np.arange(self.N+1)*self.dt*self.v_ref
        Y_ref = np.zeros(self.N+1)
        Heading_ref = np.zeros(self.N+1)
        V_ref = self.v_ref*np.ones(self.N+1)
        Z_ref = np.vstack((X_ref, Y_ref, Heading_ref, V_ref)).T
        data['ev_ref'] = Z_ref
        
        scale_mult = 1.0
        scalings = np.maximum(np.ones(self.N+1), scale_mult*np.abs(TV_v)/self.v_ref)
        crit_region = self.constraint_generator.check_collision_points(Z_ref[:,:2], TV_pred, scalings)
        hyp = [{'w': np.array([np.sign(Z_ref[i,0]),np.sign(Z_ref[i,1]),0,0]), 'b': 0, 'pos': None} for i in range(self.N+1)]
        coll_bounds = [[] for i in range(self.N+1)]
        theta = np.linspace(0, 2*np.pi, 200)
        for i in range(self.N+1):
            if crit_region[i]:
                if np.argmax(data['score'][-1]) == 0:
                    d = np.array([0,1])
                elif np.argmax(data['score'][-1]) == 1:
                    d = np.array([0,-1])
                else:
                    d = np.array([EV_x-TV_pred[i,0], EV_y-TV_pred[i,1]])
                    d = d/la.norm(d)

                hyp_xy, hyp_w, hyp_b, coll_xy = self.constraint_generator.generate_constraint(Z_ref[i], TV_pred[i], d, scalings[i])

                hyp[i] = {'w': np.concatenate((hyp_w, np.zeros(2))), 'b': hyp_b, 'pos': hyp_xy, 'coll_xy': coll_xy}
                
                coll_bound = []
                for t in theta:
                    d_bound = np.array([np.cos(t),np.sin(t)])
                    _, _, _, xy = self.constraint_generator.generate_constraint(Z_ref[i], TV_pred[i], d_bound, scalings[i])
                    coll_bound.append(xy)
                coll_bounds[i] = np.array(coll_bound)
                
        data['hyp'] = hyp
        data['coll_bound'] = coll_bounds
        data['crit_region'] = crit_region
                
        return self.image[frame_idx], self.image_t[frame_idx], frame_idx, data
        
def extract_traj(bag):
    """
    Extract traj and FSM from the bag
    """
    b = rosbag.Bag(bag)

    topics = b.get_type_and_topic_info().topics.keys()

    if len(topics) == 0:
        raise ValueError('\tNothing recorded in this bag\n')
    else:
        print("Topics contained:", topics)
        # print("Number of topics:", len(topics))

    # Initialize Dict for FSM and EV, TV
    FSM = FiniteStateMachine()
    EV = Vehicle()
    TV = Vehicle()

    # FSM
    for _, msg, t in b.read_messages('/ego_vehicle/fsm_state'):
        FSM.append_time(t.to_sec())
        FSM.append_state(msg.fsm_state)

    for _, msg, _ in b.read_messages('/ego_vehicle/strategy_scores'):
        FSM.append_score(msg.scores)

    # Current States
    for _, msg, t in b.read_messages('/ego_vehicle/est_states'):
        EV.append_state(msg, t.to_sec())
    
    for _, msg, t in b.read_messages('/target_vehicle/est_states'):
        TV.append_state(msg, t.to_sec())

    # Current Inputs
    for _, msg, t in b.read_messages('/ego_vehicle/ecu'):
        EV.append_input(msg, t.to_sec())

    for _, msg, t in b.read_messages('/target_vehicle/ecu'):
        TV.append_input(msg, t.to_sec())

    # Predicted States
    # for _, msg, _ in b.read_messages('/ego_vehicle/pred_states'):
    #     EV.append_pred_state_input(msg)
    
    # for _, msg, _ in b.read_messages('/traget_vehicle/pred_states'):
    #     TV.append_pred_state_input(msg)
    
    print("Trajectory Extraction Finished")

    cut_static_traj(FSM, EV, TV)

    return FSM, EV, TV

def extract_images(bag):
    """
    extract images from the overhead camera
    """
    b = rosbag.Bag(bag)

    topics = b.get_type_and_topic_info().topics.keys()

    if len(topics) == 0:
        raise ValueError('\tNothing recorded in this bag\n')
    else:
        print("Topics contained:", topics)

def cut_static_traj(FSM, EV, TV, threshold=1e-1):
    """
    Get rid of the static section at the begining and the end of trajectory
    """
    # From the start of trajectory
    EV_start_idx = 0
    while EV.v[EV_start_idx] < threshold:
        EV_start_idx += 1

    EV_start_time = EV.state_t[EV_start_idx]
    
    EV_end_idx = len(EV.v) - 1
    while EV.v[EV_end_idx] < threshold:
        EV_end_idx -= 1

    EV_end_time = EV.state_t[EV_end_idx]

    print("EV start time: %f, end time: %f" % (EV_start_time, EV_end_time) )

    EV.slicing_data(EV_start_time, EV_end_time)
    FSM.slicing_data(EV_start_time, EV_end_time)

    TV_start_idx = 0
    while TV.v[TV_start_idx] < threshold:
        TV_start_idx += 1

    TV_start_time = TV.state_t[TV_start_idx]

    TV_end_idx = len(TV.v) - 1
    while TV.v[TV_end_idx] < threshold:
        TV_end_idx -= 1

    TV_end_time = TV.state_t[TV_end_idx]

    print("TV start time: %f, end time: %f" % (TV_start_time, TV_end_time))

    TV.slicing_data(TV_start_time, TV_end_time)

    print("The static data are cut.")

    
