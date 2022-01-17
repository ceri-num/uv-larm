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
* a directory `grp_'color'` matching a ros package (and only this package)
* Inside the `grp_'color'` package, the code (python scripts or cpp sources) for the relevant nodes to the challenge (only the nodes developped by the students).
* The launch file `challenge1_simulation.launch` starting the appropriate nodes for demonstrating in the simulation (including the elements to start the simulation).
* The launch file `challenge1_turtlebot.launch` starting the appropriate nodes for demonstrating with a Turtlebot (including the elements to wake-up the robot).

In simulation, we will works whith configuration set in `roslaunch larm challenge-1.launch`.


### Git branching:

- [Official presentation](https://git-scm.com/book/en/v2/Git-Branching-Branches-in-a-Nutshell)

Branching permits to develop different versions of a project. Classically, in industry, you will have atleast a `dev` branch with unstable code, a `beta` branch to test and fix a solution and a `main` or `master` branch with a stable version of your project.
Additionally, you can use branching to differentiate modules, developers or client expectations in a project, etc..

For our concern here, we only want to state a stable version in the `challenge1` branch.
So, when your work is ready to be evaluated, create a new branch from your working branch and remove all the inconsistent stuff and the noise.

```bash
git checkout -b challenge1
git rm stuff
git commit -am "clean version"
git push
```

You raise an error, the first time you push a new branch.
You have to explicitly push in a new branch on the remote repository. 
(i.e. do exactly what *git* is proposing to you.)

```bash
git push --set-upstream origin challenge1
```

Then `git branch` would list all the branches on our repository and `git checkout branchName` permit to move from a branch to another.
By returning to your working branch `main` or `master`, you recover all the stuff you deleted in `challenge1` branch.
You can continue to work on your project, potentionnally


## Criteria

Minimal:

1. The group follows the consigns (i.e. the repository is presented as expected)
2. The robot behavior is safe (no collision with any obstacles)
3. The robot explore it environment

Optional:

4. Information is returned to rviz (started automaticaly) through ros geometry messages.
5. The robot movement is fluid (no stop) and fast
6. The behavior is split into several nodes (for example obstacle detection and move).
7. The group develops a specific strategy to optimize the exploration time (need to be stated in the `README.md` file)

## Evaluation protocol

Here the evaluation protocol applied.
It is highly recommended to process it yourself before the submission...

1. clone the group’s repository
1. check out the appropriate branch `git checkout challenge1`
2. Take a look to what is inside the repository and read the `README.md` file (normally it states that the project depends on `mb6-tbot`, make sure that `mb6-tbot` project is already installed aside).
3. make it: `catkin_make` and `source` from the catkin directory.
4. Launch the simulation demonstration: `roslaunch grp-color challenge1_simulation.launch` and appreciate the solution.
5. Stop everything.
6. Connect the computer to the robot and the hokuyo.
7. Launch the Turtlebot demonstration: `roslaunch grp-color challenge1_turtlebot.launch`  and appreciate the solution.
8. Take a look to the code, by starting from the launchfiles.

<!---
## In the video

1. A presentation of the challenge
2. A presentation of the launch-file and the proposed architecture
3. A demonstration
--->
