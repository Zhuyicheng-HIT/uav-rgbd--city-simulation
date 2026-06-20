from vision_people_sim_launch_common import make_launch

def generate_launch_description():
    agents = [
        {'name':'person_a','path_type':'line','points':[[-5,-1.5,0.05],[5,-1.5,0.05]],'speed':0.9},
        {'name':'person_b','path_type':'line','points':[[5,1.5,0.05],[-5,1.5,0.05]],'speed':0.7,'start_delay':0.8},
        {'name':'person_c','path_type':'rectangle','points':[[-3,-3,0.05],[3,-3,0.05],[3,3,0.05],[-3,3,0.05]],'speed':0.8,'start_delay':1.2},
        {'name':'person_d','path_type':'rectangle','points':[[-1.5,-4,0.05],[4,-4,0.05],[4,0.5,0.05],[-1.5,0.5,0.05]],'speed':0.55,'start_delay':2.0},
        {'name':'vehicle_a','path_type':'line','points':[[-6,-3.8,0.05],[6,-3.8,0.05]],'speed':1.9},
        {'name':'vehicle_b','path_type':'line','points':[[6,3.8,0.05],[-6,3.8,0.05]],'speed':1.4,'start_delay':1.0},
        {'name':'vehicle_c','path_type':'rectangle','points':[[-4.5,0.2,0.05],[4.5,0.2,0.05],[4.5,4.3,0.05],[-4.5,4.3,0.05]],'speed':1.2,'start_delay':2.0},
    ]
    return make_launch('multi_people.sdf', 'multi_people', agents)
