# MIT Principles of Autonomy and Decision Making Final Project

## Table of Contents
1. [Authors](#authors)    
2. [Windows Setup](#setup)    
    1. [Init](#init)   
    2. [Subsequent Starts](#subsequent) 
3. [Section 1](#section1)   
4. [Section 2](#section2) 
4. [Section 3](#section3)   

## Authors <a name="authors"></a>
Paul Calvetti   
<img src="https://media-exp1.licdn.com/dms/image/C4E03AQElcBMBQhM_yA/profile-displayphoto-shrink_200_200/0/1571699676588?e=2147483647&v=beta&t=wgjBfC2gBU-GL1x_W6O8xz0lwSnKKFMs7OPUWe3wECU" width="350">

Michael Cantow   
<img src="https://media-exp1.licdn.com/dms/image/D4D03AQFypps_fTAjog/profile-displayphoto-shrink_800_800/0/1670524326696?e=1675900800&v=beta&t=Y5lmI5dfs7gwtYeI2kOKQZYK_9Yn9_XK90_OaNOwHSc" width="350">

## Windows Setup <a name="setup"></a>
### Init <a name="init"></a>
Download [XLaunch]( https://sourceforge.net/projects/vcxsrv/) display server

Download [wsl](https://learn.microsoft.com/en-us/windows/wsl/install) to run ubuntu
1. wsl
3. git clone --recurse-submodules https://github.com/mcantow/padm-project.git 
4. cd padm-project
5. cd padm-project-2022f && pip install -r requirements.txt
6. cd ss-pybullet/pybullet_tools/ikfast/franka_panda/ && \
    python3 setup.py
    cd - && cd pddl-parser && \
    sudo python3 setup.py install
7. python3 -m venv .venv && source ./.venv/bin/activate
8. export DISPLAY=:0
9. pip install numpy
10. pip install pybullet
11. pip install gitmodules
12. pip install padm-project-2022f/pddl-parser
11. cd padm-project-2022f/ && python minimal_example.py

### Subsequent Starts <a name="subsequent"></a>
1. wsl
2. cd padm-project
3. ./.venv/bin/activate


## Section 1 <a name="section1"></a>
### Assumptions made when designing domain
When designing our domain, we allowed our actions to be relatively general. For example, the pickupFrom{X} actions require the robot arm to be clear and an object to be at the source position. We assume that given these preconditions, the robot arm knows how to navigate to these locations and squeeze the object. In future stages, these general actions may need to be combinations of sub actions, like navigate to position, squeeze claw, lift from position, etc. We also assume there is only one robot arm, but this assumption could be relaxed with a few abstractions in our problem and domain files.

### Approach to generate plan
We define the problem initial and goal conditions, as well as the objects in the problem.pddl file. We define the various actions that we can take in the domain.pddl file. 

In the activity planner.py file, we do the work of finding a path from the initial state to the goal state by utilizing the allowed actions. We take advantage of the PDDL_Parser library included in the initial code distribution. Using this, we essentially perform a BFS of our space. More specifically, starting with the start state, we find all actions that are valid (using our get\_available\_actions function). For each of these actions, we generate a new state (using our update\_state function) which reflects what our state is after executed a certain action from the previous state. We repeat this process until we either exhaust our search space, or find a solution. We use a helper function called is\_goal to check if we are in the goal state. It checks if the state contains all the positive goal conditions and none of the negative goal conditions.

I will also describe a few of the helper functions used in our implementation:

Our get\_available\_actions function essentially takes in the current state and a list of all the potential actions. It checks if the state has all positive preconditions of the action and none of the negative preconditions of the action. If this is the case, then that action is added to the list of potential actions.

Our update\_state function take in an initial state and action and returns an updated state based on what it would be after the execution of that action. The way this is done is it takes the union of the add effects of the action and the current conditions of the state. It then also adds in any conditions of the current state which are not in the delete effects of the action.

### Challenges
We had issues getting the simulator to run. We are both working on windows, and ultiimately one of us is running the windows setup described above and the other is running an ubuntu vm. We found working with the pddl library to be pretty straightforward and our custom planner is working as expected.

Another challenge that we are still dealing with is using types in the pddl definition. It seems like doing so would make our code easier to read and would potentially offer runtime improvements. We experimented with using types, but couldn't successfully get the code to work using them at this point.


## Section 2 <a name="section2"></a>
### Assumptions made when designing domain
First, we hard coded the base to move near the objects we need to interact with. Since these objects are all reachable from a fixed base point, we deterministically move there to reduce complexity when we later run RRT. We also hard coded the objects to move with the arm. Once the arm is within epsilon of an object and we run a grab action, this object is hard coded to move with the arm.

### Explain the files and the motion planners you implemented
The new files we added were run_simulation.py and initial_simulation.py. run_simulation contains all the code for this section, its main function executes the simulation showing the motion plan for each of the activity plan actions.

### Explain how you integrated the activity plan with the motion plan
The run_simulation file calls our activity planner, which turns a list of strings representing actions to execute the objectives. For each of these actions, our planner maps the string to a function that computes a goal position. It then runs a custom implementation of RRT based on the position of the arm with collision checking to generate a motion plan consisting of arm positions. The plans are then concatenated and executed. 

Our RRT implmented the general algorithm presented in class, but it was a bit more complicated because we had to go between the joint positions and arm positions. Essentially we would randomly generate joint positions using the "sample\_fn" given in the minimal example. We would then find the associated pose and try to move between the randomly generated pose and the clostest one. We used the "closest\_inverse\_kinematics" and "interpolate\_poses" functions of pybullet to find the poses and joint assingments in the path between the poses. Then for each of those we would check for collisions (use a helper function the utilized the built in "pairwise_collision" function) We would repeat this to construct a larger tree. Every 20 loops we would then try to go from each node in our tree (an assignment of joint positions) to the goal pose.  

### Gif
![gif-broken](https://github.com/mcantow/padm-project/blob/main/unoptimized.gif)

### Challenges
We originally had just a open drawer function, and not a go to drawer function. Because of the way the motion planner ended up getting coded, it made the most sense to add a go to drawer function before the open drawer function. We had to make the according change in our domain and problem files, which made this step trivially integrate with the code to move the boxes. 

We found pybullet difficult to work with. There were several thousands of lines of code with no documentation. The minimal_example.py provided was useful but we still found the experience somewhat cumbersome. 


## Section 3 <a name="section3"></a>
### Explain the files, key functions, and the solver you used
In this section, we added 4 main files: optimize_trajectory.py, optimize_run_simulation.py, actions.pickle, and opt_actions.pickle. optimize_trajectory.py contains a class, trajectoryOptimizer, which manages how we actually run the trajectory optimization. The file also contains a function, debug, that shows how to call the trajectoryOptimizer class to optimize a trajectory from our motion plan. optimize_run_simulation.py contains a class, optimized_simulation, which inherits from the simulation class used to run the unoptimized RRT simulation in section 2. This child class contains a method, optimize_simulation_step(action), which uses the optimizer in optimize_trajectory.py to optimize the place object in drawer action generated by the motion plan in section 2, then runs the simulation. The other methods of this class are used to pickle/unpickle results. The pickle files save the motion plans for the unoptimized and optimized actions respectively, which we found useful during debugging since the motion planning piece took a long time to run. We used the pydrake implementation of the KinematicTrajectoryOptimization class in optimize_trajectory.py to compute the optimized motion plan. We used a BsplineBasis of order 4 to create the BsplineTrajectory, set constraints, then solved using pydrake.solvers.Solve. 

### Explain what optimization problem you are trying to solve and why


### Formalize the constrained optimization problem using mathematical symbols and relationships 
Input $X = [X_1, X_2, ...,X_n]$ where $X_i$ is the $i$ith assignment of joint positions in our non-optimized path.
Output $Y = [Y_1, Y_2, ...,Y_n]$ where $X_i$ is the $i$ith assignment of joint positions in our non-optimized path.

<ins>Constraints</ins>

Start and end positions are the same as our sample trajectory

$Y_1 == X_1$

$Y_n == X_n$

Jerk bound

$\left((Y_i[j] - Y_{i+1}[j]) - (Y_{i+1}[j] - Y_{i+2}[j])\right) - \left((Y_{i+1}[j] - Y_{i+2}[j]) - (Y_{i+2}[j] - Y_{i+3}[j])\right) == 0 \forall  i \in \{1,2,...,n-3\}, j\in\{1,2,3,4,5,6,7\}$

<ins>Objective</ins>

Minimize the l2 distance of the joint movement.

$\underset{Y}{\operatorname{argmin}} \sum_{i=1,...,n-1} |Y_i-Y_{i-1}|_2$


### Gif
![gif-broken](https://github.com/mcantow/padm-project/blob/main/optimized.gif)

### Challenges
Before we implemented the functionality to pickle our motion plans and load them when working on the optimization step, we were constantly rerunning RRT to generate a motion plan. This was taking a long time (~1 minute) to run and made debugging extremely slow. Once we realized we could save these results and load them very quickly, our workflow became much simpler and our laptops cooled down. We would recommend suggesting this to students in future project years. 

When framing the optimization problem, we found it fairly intuitive to set the initial and goal position constraints by using AddPathPositionConstraint at the beginning and end of the trajectory with the position our motion planner generated. We also found it intuitive to AddPathLengthCost() to optimize over the trajectory path length. We ran into issues with the solver taking very long to run with those constraints. Eventually we realized that this was due to unconstrained variables, so to resolve that issue we constrained Jerk to be zero throughout the trajectory. This resolved the issue and generated a nice path.

Lastly, we chose to optimize the place object in drawer action because it did not have issues with obstacles. We could have added additional position constraints for the other actions, but we decided that this action was very suboptimal and we could reasonably frame its optimization problem given the time allotted. 


### Compare the resulting optimized trajectory to the initial sample-based motion plan
Our original trajectories are mostly very suboptimal with the exception of moving the box to the counter, which was a simple operation. This makes sense given we computed them with RRT, randomly exploring over the arm position space and taking the first collision free trajectory we could find. We optimized the action of moving the can from the counter to the drawer, which originally took a long unnecessary upward arc. After our optimization, the arm takes a pretty direct path from source to goal. This can be visualized by comparing the gif in section 2 to the gif in section 3 for that actions. Note we did not optimize the other actions.
