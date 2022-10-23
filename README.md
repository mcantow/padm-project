# MIT Principles of Autonomy and Decision Making Final Project

## Table of Contents
1. [Authors](#authors)    
2. [Windows Setup](#setup)    
    1. [Init](#init)   
    2. [Subsequent Starts](#subsequent) 
3. [Section 1](#section1)   

## Authors <a name="authors"/>
Paul Calvetti   
<img src="https://media-exp1.licdn.com/dms/image/C4E03AQElcBMBQhM_yA/profile-displayphoto-shrink_200_200/0/1571699676588?e=2147483647&v=beta&t=wgjBfC2gBU-GL1x_W6O8xz0lwSnKKFMs7OPUWe3wECU" width="350">

Michael Cantow   
<img src="https://media-exp1.licdn.com/dms/image/C4E03AQF7X7g3Wi1oOw/profile-displayphoto-shrink_800_800/0/1546477526522?e=1671667200&v=beta&t=1cLVm9-B8OH91wv42ia9ydE69XMCqE-icNQarMlzQzQ" width="350">

## Windows Setup <a name="setup"/>
### Init <a name="init"/>
Download [XLaunch]( https://sourceforge.net/projects/vcxsrv/) display server

Download [wsl](https://learn.microsoft.com/en-us/windows/wsl/install) to run ubuntu
1. wsl
3. git clone --recurse-submodules https://github.com/mcantow/padm-project.git 
4. cd padm-project
5. python3 -m venv .venv && source ./.venv/bin/activate
6. export DISPLAY=:0
7. pip install numpy
8. pip install pybullet
9. cd padm-project-2022f/ && python minimal_example.py

### Subsequent Starts <a name="subsequent"/>
1. wsl
2. cd padm-project
3. ./.venv/bin/activate


## Section 1 <a name="section1"/>
### Assumptions made when designing domain
When designing our domain, we allowed our actions to be relatively general. For example, the pickupFrom{X} actions require the robot arm to be clear and an object to be at the source position. We assume that given these preconditions the robot arm knows how to navigate to these locations and squeeze the object. In future stages, these general actions may need to be combinations of sub actions, like navigate to position, sqeeze claw, lift from position, etc.

### Approach to generate plan


### Challenges
We had issues getting the simulator to run. We are both working on windows, and ultiimately one of us is running the windows setup described above and the other is running an ubuntu vm. We found working with the pddl library to be pretty straightforward and our custom planner is working as expected.
