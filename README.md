# LARM - Logiciel et Architecture pour la Robotique Mobile - Software and Architecture for Mobile-Robotics

Here a lecture on software and architecture for mobile robots at the [IMT-Nord](http://www.imt-nord-europe.fr) engineering school.

This lecture is an introduction on how to develop a modular control and supervision software for a mobile platform.
The notions are illustrated with tutorials based on Linux/[ROS](https://ros.org/) and [Kobuki/Turtlebot2](http://kobuki.yujinrobot.com/) robot.

Introduction - [pdf version](https://raw.githubusercontent.com/ceri-num/uv-larm/master/notions/sld-intro.pdf)

## This support

This support is stored on [github](https://github.com) shared thanks to [gitbook](https://www.gitbook.com).

  - On gitbook: <https://ceri-num.gitbook.io/uv-larm/>
  - On GitHub: <https://github.com/ceri-num/uv-larm>

<!--
## Tutorials

Due to the pandemic extraordinary situation, this course is largely based over [TheConstruct](https://www.theconstructsim.com/) solution.
TheConstruct provides courses materials and more importantly virtualized ROS machine allowing you to develop robotics software based on Gazebo simulation.

<!--
This lecture is composed of multiple guided tutorials (non-exhaustive list):

* Setting up your own ROS environment
* Moving a robot
* Simulatenous Localization and Mapping (SLAM)
* Autonomous Navigation
* Robotics Vision
-->

## Challenge

The evaluation mainly consists in the realization of an application involving specific challenges:

- Autonomous Control of an **AGV** (Automated Guided Vehicle)
- Mapping and Localization
- Research and recognition of an object

2022-2023 groups: [on partage](https://partage.imt.fr/index.php/s/zkQbXMsrWdp2RQd)

## Going further

Most of the content and supports for learning robotics architecture are already shared on the internet.
We try to orient the students through project realizations rather than to provide an exhaustive definition of concepts and implementations.
This course relies on the [ROS](http://www.ros.org/) middleware for practical sessions, the ROS wiki tutorials and ros-packages' descriptions: <https://wiki.ros.org/>.

You also can find an excellent virtual working environment and resources on [TheConstruct](https://www.theconstructsim.com/).

## Presentations

Slide generation is based on [marp](https://marp.app/) and the associated puggin in [VS Code](https://marketplace.visualstudio.com/items?itemName=marp-team.marp-vscode).

To attach the appropriate style, go in `VS Code` parameters (Bottom left corner) - search for `marp:theme` - select the workspace tab - add element `style/imt.css`.
Then, search `marp:html` and enable all html elements.

## Contact

For comments, questions, corrections, feel free to contact:

[Guillaume Lozenguez](mailto://guillaume.lozenguez@imt-nord-europe.fr) (coordinator, but not the only author here).
