# MIT Principles of Autonomy and Decision Making Final Project

## Table of Contents
1. [Authors](#authors)    
2. [Windows Setup](#setup)    
    1. [Init](#init)   
    2. [Subsequent Starts](#subsequent) 
3. [Section 1](#section1)   
4. [Section 2](#section2)   

## Authors <a name="authors"></a>
Paul Calvetti   
<img src="https://media-exp1.licdn.com/dms/image/C4E03AQElcBMBQhM_yA/profile-displayphoto-shrink_200_200/0/1571699676588?e=2147483647&v=beta&t=wgjBfC2gBU-GL1x_W6O8xz0lwSnKKFMs7OPUWe3wECU" width="350">

Michael Cantow   
<img src="https://media-exp1.licdn.com/dms/image/C4E03AQF7X7g3Wi1oOw/profile-displayphoto-shrink_800_800/0/1546477526522?e=1671667200&v=beta&t=1cLVm9-B8OH91wv42ia9ydE69XMCqE-icNQarMlzQzQ" width="350">

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


## Section 2 <a name="section1"></a>
### Assumptions made when designing domain
First, we hard coded the base to move near the objects we need to interact with. Since these objects are all reachable from a fixed base point, we deterministically move there to reduce complexity when we later run RRT. We also hard coded the objects to move with the arm. Once the arm is within epsilon of an object and we run a grab action, this object is hard coded to move with the arm.

### Explain the files and the motion planners you implemented
The new files we added were run_simulation.py and initial_simulation.py. run_simulation contains all the code for this section, its main function executes the simulation showing the motion plan for each of the activity plan actions.

### Explain how you integrated the activity plan with the motion plan
The run_simulation file calls our activity planner, which turns a list of strings representing actions to execute the objectives. For each of these actions, our planner maps the string to a function that computes a goal position. It then runs a custom implementation of RRT based on the position of the arm with collision checking to generate a motion plan consisting of arm positions. The plans are then concatenated and executed. 

### Gif
![gif-broken](https://github.com/mcantow/padm-project/blob/main/video3661558971.gif)

### Challenges
We originally had just a open drawer function, and not a go to drawer function. Because of the way the motion planner ended up getting coded, it made the most sense to add a go to drawer function before the open drawer function. We had to make the according change in our domain and problem files, which made this step trivially integrate with the code to move the boxes. 

We found pybullet difficult to work with. There were several thousands of lines of code with no documentation. The minimal_example.py provided was useful but we still found the experience somewhat cumbersome. 
