# Random exploration of a Small Environment

The goal of the challenge is to demonstrate the capability of a robot to move in a cluttered environment and potentially to visit all a closed area.

<!--
## Preparation

Install the [LARM simulation workspace](https://github.com/ceri-num/LARM-RDS-Simulation-WS) in place of `simulation_ws`.
-->

## Expected

The robot is positioned somewhere in a closed area (i.e. an area bounded with obstacles).

The robot moves continuously in this area by avoiding the obstacles (i.e. area limits and some obstacles randomly set in the area).

The robot trajectory would permit the robot to explore the overall area. In other words, the robot will go everywhere (i.e. the probability that the robot will reach a specific reachable position in the area is equal to 1 at infinity times).

One first approach can be to develop a *ricochet robot* that changes its direction each time an obstacle prevent the robot to move forward.


## consigns

Each group commit the minimal required files in a specific `challenge1` git branch.

### The required files:

* a `README.md` file in markdown syntax introducing the project.
* The ros package files of a `grp-color` ros package (and only this package)
* The code (python script or cpp sources) for the relevant nodes to the challenge.
* The launch file `challendge1_simulation.launch` starting the appropriate nodes demonstrating in the simulation (including the simulation).
* The launch file `challendge1_turtlebot.launch` starting the appropriate nodes demonstrating with a turtle bot (including robot wake-up).


### Git branching:

- [Official presentation](https://git-scm.com/book/en/v2/Git-Branching-Branches-in-a-Nutshell)

Branching permits to develop different versions of a project. Classically you will have a `dev` branch with unstable code, a `beta` branch to test and fix a solution and a `main` or `master` branch with a stable version of your project. You can use branching to differentiate developers or clients in a project, etc..

For our concern here, we only want to state a stable version in the `challendge1` branch. So, when your work is done, create a new branch from your working branch and remove all the inconsistent stuff and the noise.

```bash
git checkout -b challendge1
git rm stuff
git commit -am "clean version"
git push
```

Then `git branch` would list all the branches on our repository and `git checkout branchName` permit to return to your working branch and recover all the stuff you deleted in `challendge1` branch.


## Criteria

Minimal:

1. The group follows the consigns (i.e. the repository is presented as expected)
2. The robot behavior is safe (no collision with any obstacles)
3. The robot explore it environment

Optional:

4. Information is returned to rviz through ros geometry messages.
5. The robot movement is fluid (no stop) and fast
6. The group develops a specific strategy to optimize the exploration time

## Evaluation protocol

Here the evaluation protocol applied. It is highly recommended to process it yourself before the submission...

1. clone the group’s repository
1. checkout the appropriate branch `git checkout challendge1`
2. make it `catkin_make` from the catkin directory.
3. Launch the simulation demonstration: `roslaunch grp-color challendge1_simulation.launch`
4. Stop everithing.
5. Connect the computer to the robot.
6. Launch the Turtlebot demonstration: `roslaunch grp-color challendge1_turtlebot.launch`
7. Read the `README.md` file and take a look to the code.

<!---
## In the video

1. A presentation of the challenge
2. A presentation of the launch-file and the proposed architecture
3. A demonstration
--->
