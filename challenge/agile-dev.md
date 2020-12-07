# Working with iterations

The clear objective for a team of developers intents more efficient software development.
The efficiency is relative to the quality of the developed software, its documentation, its code and the time needed to achieve all of that.

The developer observed in its natural environment dives deeply in its source code until the very end of a dead line.
It is then too late to stick together the among of developed functionality in an ergonomic way, to generate documentation and to state that the direction of the project does not match the initial expectations. 

### Iterative development


The main feature consists of iterative development
by moving forward incrementally
and by delivering operational versions frequently.

For this project, development is split in two iterations.
A first action would be to prepare a board to help the team on it organization.

This board is composed by **6** areas:

- *Team name*: and developers' name.
- *to-do - 1st it*: List of tasks to perform before the end of the first iteration (**the 7th of January**)
- *to-do - 2d it*: List of tasks to perform before the end of the second and last iteration (**the 15th of January**)
- *Doing*: List of tasks the team is working on
- *Waiting*: List of tasks waiting for an external input
- *Donne*:  List of terminated tasks

![](resources/team-board.svg)

A team is composed of:

- **The developers** (2 or 3)
- **The external observers** (teachers)
- **The colleges** <br /> (they help because they aim you succeed (but not as the winner), and they have their own project to work on)



### Project decomposition

Then the project is decomposed in several components to develop in order to reach the project objectives.
The main objective is to fill the team board with the components to develop, at least for the first iteration.

A component need to be

- *Idenpedent*: It development do not require that another component to be developed at the same time.
- *Verifiable*: It is possible to test if the component work correctly.
- *Complete*: Each time a component is terminated, the project status is operational and could be released.

Define the components to develop, at least to reach a first milestone: *"a robot capable of navigating in an environment with obstacles"*.
Ideally, each component matches a functionality for the robot, implemented as a *ROS* node.
Select in the *to-do - 1st it* area of your *team board* all the components you will develop during the first iteration.
Let the other component for the second iteration.
To notice that, the component to develop during the second iteration would be refined at the end of the first iteration.

