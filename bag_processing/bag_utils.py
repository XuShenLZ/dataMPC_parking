import rosbag
import numpy as np
import matplotlib.pyplot as plt

class Vehicle(object):
    """
    Vehicle Class as EV / TV data structure
    """
    def __init__(self):
        """
        Initializing
        """
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
        self.t.append(t)
    
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
    FSM = {"state": [], "score": []}
    EV = Vehicle()
    TV = Vehicle()

    # FSM
    for _, msg, _ in b.read_messages('/ego_vehicle/fsm_state'):
        FSM["state"].append(msg.fsm_state)

    for _, msg, _ in b.read_messages('/ego_vehicle/strategy_scores'):
        FSM["state"].append(msg.scores)

    # Current States
    for _, msg, t in b.read_messages('/ego_vehicle/est_states'):
        EV.append_time(t)
        EV.append_state(msg)
    
    for _, msg, t in b.read_messages('/target_vehicle/est_states'):
        TV.append_time(t)
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

    
