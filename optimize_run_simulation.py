from run_simulation import *
from optimize_trajectory import trajectoryOptimizer

class optimized_simulation(simulation):

    def optimize_simulation_step(self, action):
        '''
        Modify self.action_to_robot_joint_positions_dict 
        for one of the actions with an optimized trajectory
        '''
        print('Optimizing trajectory of {}'.format(action))
        trajectoryToOptimize = self.action_to_robot_joint_positions_dict[action]
        optimizer = trajectoryOptimizer(trajectoryToOptimize)
        result = optimizer.solveProblem()
        optimizedTrajectory = result
        self.action_to_robot_joint_positions_dict[action] = optimizedTrajectory

if __name__ == '__main__':
     sim = optimized_simulation()
     sim.make_activity_plan()
     sim.move_robot_into_position()
     sim.plan_simulation()
     for action in sim.action_to_robot_joint_positions_dict.keys():
        print(action, len(sim.action_to_robot_joint_positions_dict[action]))
     sim.optimize_simulation_step('pickupfromtable') # TRAJECTORY OPTIMIZATION
     sim.run_simulation()
