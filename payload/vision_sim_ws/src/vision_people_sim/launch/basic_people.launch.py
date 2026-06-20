from vision_people_sim_launch_common import make_launch

def generate_launch_description():
    agents = [
        {'name':'person_line','path_type':'line','points':[[-4.0,2.2,0.05],[4.0,2.2,0.05]],'speed':0.8,'distance_offset':0.0},
        {'name':'person_rect','path_type':'line','points':[[-4.0,-1.2,0.05],[4.0,-1.2,0.05]],'speed':0.8,'start_delay':2.0,'distance_offset':4.0},
        {'name':'vehicle_line','path_type':'line','points':[[-6.0,-3.6,0.05],[6.0,-3.6,0.05]],'speed':1.6,'distance_offset':3.0},
    ]
    return make_launch('basic_people.sdf', 'basic_people', agents)
