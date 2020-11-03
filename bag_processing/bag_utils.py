import rosbag
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import warnings

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

class VideoReader(object):
    def __init__(self, filename, image_topic='/overhead_camera/image_rect_color'):
        bridge = CvBridge()
        
        self.image_ts = []
        self.images = []
        with rosbag.Bag(filename, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == image_topic:
                    try: 
                        cv_image = bridge.imgmsg_to_cv2(msg, 'rgb8')
                    except CvBridgeError, e:
                        print(e)
                    self.image_ts.append(t.to_sec())
                    self.images.append(cv_image)
        
        self.image_ts = np.array(self.image_ts)
        self.video_start = np.amin(self.image_ts)
        self.video_end = np.amax(self.image_ts)
        self.video_length = self.video_end - self.video_start
        self.n_frames = len(self.image_ts)
        
        print('Video of duration %g s read in as %i frames' % (self.video_length, self.n_frames))
    
    def get_frame(self, time=None, frame=None):
        if time is None and frame is None:
            raise(RuntimeError('Need to specify a time between up to %g s or a frame number between up to %i' % (self.video_length, self.n_frames-1)))
        if time is not None and frame is not None:
            raise(RuntimeError('Can only specify either time or frame'))
            
        if time is not None:
            assert time >= 0, 'Time must be non-negative'
            if time > self.video_length:
                warnings.warn('Desired time of %g s is greater than video length of %g s, returning last frame' % (time, self.video_length))
                time = self.video_length
            frame = np.argmin(np.abs(self.image_ts - (time+self.video_start)))
        elif frame is not None:
            assert frame >= 0, 'Frame number must non-negative'
            if frame >= self.n_frames:
                warnings.warn('Desired frame number of %i is greater than %i, returning last frame' % (frame, self.n_frames))
                frame = self.n_frames[-1]
        
        return self.images[frame], self.image_ts[frame], frame
        
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

    
