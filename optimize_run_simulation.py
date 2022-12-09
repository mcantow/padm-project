import pydrake
from run_simulation import *

class optimized_simulation(simulation):

    def optimize_simulation_step(self, action):
        '''
        Modify self.action_to_robot_joint_positions_dict 
        for one of the actions with an optimized trajectory
        '''
        trajectoryToOptimize = self.action_to_robot_joint_positions_dict['pickupfromtable']
        # REVERSING ACTION AFTER EXECUTION TO DEMONSTRATE HOW STRUCTURE WORKS
        # TODO OPTIMIZE TRAJECTORY HERE 
        optimizedTrajectory = trajectoryToOptimize + trajectoryToOptimize[::-1]
        # END TODO 
        self.action_to_robot_joint_positions_dict['pickupfromtable'] = optimizedTrajectory

if __name__ == '__main__':
     sim = optimized_simulation()
     sim.make_activity_plan()
     sim.move_robot_into_position()
     sim.plan_simulation()
     for action in sim.action_to_robot_joint_positions_dict.keys():
        print(action, len(sim.action_to_robot_joint_positions_dict[action]))
     sim.optimize_simulation_step('CHANGE_ME') # TRAJECTORY OPTIMIZATION
     sim.run_simulation()
