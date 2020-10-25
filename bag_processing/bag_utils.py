import rosbag
import numpy as np

class Vehicle(object):
    """
    Vehicle Class as EV / TV data structure
    """
    def __init__(self):
        """
        Initializing
        """
        self.t0 = None
        self.t = []

        self.x = []
        self.y = []
        self.heading = []
        self.v = []

        self.delta = []
        self.a = []

        self.pred = None

    def append_time(self, t):
        """
        append current time
        """
        if self.t0 == None:
            self.t0 = t
            
        self.t.append(t-self.t0)
    
    def append_state(self, msg):
        """
        append vehicle state
        """
        self.x.append(msg.x)
        self.y.append(msg.y)
        self.heading.append(msg.psi)
        self.v.append(msg.v)

    def append_input(self, msg):
        """
        append vehicle input
        """
        self.delta.append(msg.servo)
        self.a.append(msg.motor)

    def append_pred_state_input(self, msg):
        """
        append predicted vehicle state and input
        """
        if self.pred == None:
            self.pred = Vehicle()
        
        self.pred.append_state(msg)

        self.pred.delta.append(msg.df)
        self.pred.a.append(msg.a)

    def slicing_states(self, start_idx, end_idx):
        """
        take a slice of the time and states
        """
        self.t0 = self.t[start_idx]
        self.t = [x-self.t0 for x in self.t[start_idx: end_idx]]

        self.x = self.x[start_idx: end_idx]
        self.y = self.y[start_idx: end_idx]
        self.heading = self.heading[start_idx: end_idx]
        self.v = self.v[start_idx: end_idx]

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
        self.state_names = ["Free-Driving", "Safe-Confidence",
                            "Safe-Yield", "Safe-Infeasible",
                            "HOBCA-Unlocked", "HOBCA-Locked",
                            "Emergency-Break"]

        self.score_l = []
        self.score_r = []
        self.score_y = []

    def append_time(self, t):
        """
        append time
        """
        if self.t0 == None:
            self.t0 = t
        
        self.t.append(t - self.t0)

    def append_state(self, state):
        """
        append state and convert it into index
        """
        self.states.append(state)

        idx = self.state_names.index(state)
        # if state == "Free-Driving":
        #     idx = 0
        # elif state == "Safe-Confidence":
        #     idx = 1
        # elif state == "Safe-Yield":
        #     idx = 2
        # elif state == "Safe-Infeasible":
        #     idx = 3
        # elif state == "HOBCA-Unlocked":
        #     idx = 4
        # elif state == "HOBCA-Locked":
        #     idx = 5
        # elif state == "Emergency-Break":
        #     idx = 6

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

        self.t0 = self.t[start_idx]
        self.t = [x-self.t0 for x in self.t[start_idx: end_idx]]

        self.states = self.states[start_idx:end_idx]
        self.state_idxs = self.state_idxs[start_idx:end_idx]

        self.score_l = self.score_l[start_idx:end_idx]
        self.score_r = self.score_r[start_idx:end_idx]
        self.score_y = self.score_y[start_idx:end_idx]


def extract_traj(bag):
    """
    Extract traj and FSM from the bag
    """
    b = rosbag.Bag(bag)

    topics = b.get_type_and_topic_info().topics.keys()

    if len(topics) == 0:
        raise ValueError('\tNothing recorded in this bag\n')
    else:
        # print("Topics contained:", topics)
        print("Number of topics:", len(topics))

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
        EV.append_time(t.to_sec())
        EV.append_state(msg)
    
    
    for _, msg, t in b.read_messages('/target_vehicle/est_states'):
        TV.append_time(t.to_sec())
        TV.append_state(msg)

    # Current Inputs
    for _, msg, _ in b.read_messages('/ego_vehicle/ecu'):
        EV.append_input(msg)

    for _, msg, _ in b.read_messages('/target_vehicle/ecu'):
        TV.append_input(msg)

    # Predicted States
    for _, msg, _ in b.read_messages('/ego_vehicle/pred_states'):
        EV.append_pred_state_input(msg)
    
    for _, msg, _ in b.read_messages('/traget_vehicle/pred_states'):
        TV.append_pred_state_input(msg)
    
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

    EV_start_time = EV.t[EV_start_idx]
    
    EV_end_idx = len(EV.v) - 1
    while EV.v[EV_end_idx] < threshold:
        EV_end_idx -= 1

    EV_end_time = EV.t[EV_end_idx]

    EV.slicing_states(EV_start_idx, EV_end_idx)
    FSM.slicing_data(EV_start_time, EV_end_time)

    TV_start_idx = 0
    while TV.v[TV_start_idx] < threshold:
        TV_start_idx += 1

    TV_end_idx = len(TV.v) - 1
    while TV.v[TV_end_idx] < threshold:
        TV_end_idx -= 1

    TV.slicing_states(TV_start_idx, TV_end_idx)

    print("The static data are cut.")

    
