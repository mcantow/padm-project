from run_simulation import *
from optimize_trajectory import trajectoryOptimizer
import pickle

class optimized_simulation(simulation):

    def load_simulation_steps(self):
        with open('actions.pickle', 'rb') as handle:
            actions = pickle.load(handle)
            self.action_to_robot_joint_positions_dict = actions
            return actions

    def load_opt_simulation_steps(self):
        with open('opt_actions.pickle', 'rb') as handle:
            actions = pickle.load(handle)
            self.action_to_robot_joint_positions_dict = actions
            return actions

    def pickle_simulation_steps(self):
        actions = self.action_to_robot_joint_positions_dict
        with open('actions.pickle', 'wb') as handle:
            pickle.dump(actions, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def pickle_opt_simulation_steps(self):
        actions = self.action_to_robot_joint_positions_dict
        with open('opt_actions.pickle', 'wb') as handle:
            pickle.dump(actions, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def optimize_simulation_step(self, action):
        '''
        Modify self.action_to_robot_joint_positions_dict 
        for one of the actions with an optimized trajectory
        '''
        print('Optimizing trajectory of {}'.format(action))
        self.action_to_robot_joint_positions_dict = self.load_simulation_steps()
        trajectoryToOptimize = self.action_to_robot_joint_positions_dict[action]
        optimizer = trajectoryOptimizer(trajectoryToOptimize)
        result = optimizer.solveProblem()
        optimizedTrajectory = result
        self.action_to_robot_joint_positions_dict[action] = optimizedTrajectory
        self.pickle_opt_simulation_steps()


if __name__ == '__main__':
     sim = optimized_simulation()

     ## plan unoptimized simulation and pickle results
    #  sim.make_activity_plan()
    #  sim.move_robot_into_position()
    #  sim.plan_simulation()
    #  sim.pickle_simulation_steps()

     ## optimize the saved action plan
     sim.optimize_simulation_step('putindrawer') # TRAJECTORY OPTIMIZATION

     ## run non optimized simulation
    #  sim.move_robot_into_position()
    #  sim.make_activity_plan()
    #  sim.load_simulation_steps()
    #  sim.run_simulation()

     ## run optimized simulation
    #  sim.move_robot_into_position()
    #  sim.make_activity_plan()
    #  sim.load_opt_simulation_steps()
    #  sim.run_simulation()

    #  for action in sim.action_to_robot_joint_positions_dict.keys():
    #     print(action, len(sim.action_to_robot_joint_positions_dict[action]))
    #  sim.optimize_simulation_step('pickupfromtable') # TRAJECTORY OPTIMIZATION
    #  sim.run_simulation()
