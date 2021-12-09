---
marp: true
theme: imt
paginate: true
backgroundImage: url('../style/bg-imt.svg')
---

# UV LARM 
### Logiciel et Architecture pour la<br />Robotique Mobile
### An introduction

<br />
<br />

**Guillaume Lozenguez**
[@imt-nord-europe.fr](mailto:guillaume.lozenguez@imt-nord-europe.fr)

![bg](../style/bg-tittle.svg)

---

1. **What is a Robot ?**
2. **About the Lecture LARM**
3. **Today: First contact with Linux and ROS**

![bg](../style/bg-toc.svg)

---
<!-- --------------------------------------------------------------- -->

## What is a Robot ?

### On Wikipedia:

**en**

"A robot is a machine—especially one programmable by a computer— capable of carrying out a complex series of actions automatically."

**fr**

"Un robot est un dispositif mécatronique (alliant mécanique, électronique et informatique) conçue pour accomplir automatiquement des tâches imitant ou reproduisant, dans un domaine précis, des actions humaines."

---

## What is a Robot ?

### From my point of view

"A **robot** is a **mechatronics** machine capable of autonomously acting in a real environment."

- perceives with *sensors*
- models its environment and adapts its behavior
- acts with *actuators*

generally involves Artificial-Intelligence:

- capable to mimic natural (human, animal, insect,...) intelligence

---

## Some examples

![width:700](../files/intro/exemple_robot.svg)

Macro: a large variety of robots

---

## Some examples


![width:700](../files/intro/exemple_components.svg)

Micro: a large variety of components.

---

## From a mechanic point of view

<div class="one2">

### Focus on:

- Resistance
- Weight
- Distortion
- Vibration absorption
- Machining, Assembly

</div>
<div class="one2">

### for different robots:

- Fast
- Precise
- Strong
- resistant (dust, water,...)
- safe
- less expensive

</div>

*From an electronic point of view*

Focus on sensors, motor, energy systems and hardware.

---

## From a automation point of view

### Focus on:

- Physics science
- Signal processing
- Control system

### by manipulating

- Times series, torques
- Vector, Matrices

---

## From a software point of view

### Focus on:

- Algorythms
- Knowledge representation
- Artificial intelligence
- Software architecture

Robots are complex and singular systems <br />
which require modular computer programs.

---

1. **What is a Robot ?**
2. **About the Lecture LARN**
3. **Today: First contact with Linux and ROS**

![bg](../style/bg-toc.svg)

---

## UV-LARM

#### Software and Architecture for Mobile Robots

**Mostly about:** autonomous navigation.

- Communicate with robot components
- Control robot movements (nonholonomic robot)
- Perception of the local environment (laser, vision)
- SLAM (Simultaneous Localization and Mapping)
- Path finding and navigation.

---

## UV-LARM - Schedule

<br />

*1st week:* Introduction to notions with tutorials.

*2d week:* Challenge kickoff and some complementary notions.

*3d week:* Challenge as your projet.

*4th week:* Evaluation through the code you provide.

- Always from *9:00* to *12:00* and from *14:00* to *17:30*.
- In *Develter* and *3005*.
- With or without a teacher.

---

## Why ROS:

**ROS:** The Robot Operating System (ROS) is a set of *software libraries* and *tools* that help you build robot applications.

- The number one Robotic Middle used in academic
- Open and oriented toward its *many contributors*
- Supported by most of the professionals

It permits thinking robotic programs in a modular way as independent program's *nodes* working together by communicating through *topics*.

It comes with useful functionality like *frame* management and *transform*

---

## Why Ubuntu Linux:

**Because**

- We love *GNU*
- ROS supports natively Ubuntu Linux
- _And mainly:_ Linux is efficient, open and well documented


---

## Evaluation:



---

1. **What is a Robot ?**
2. **About the Lecture LARN**
3. **Today: First contact with Linux and ROS**

![bg](../style/bg-toc.svg)


---

## Today:

### [wiki ROS](http://wiki.ros.org/) Beginner tutorials:

- Create a ROS project (catkin)
- Implement communicating nodes (publisher and subscribers)

### But first : Installation and configuration of Ubuntu:

- Setup tutorial on gitbook: <https://ceri-num.gitbook.io/uv-larm/>

---

## Before to go :

### Reminder on Linux Terminal (or Shell)

- **ls**: list directories elements
- **cd**: change the directory
- **rm**: permanently remove a file
- **man**: open the manual on a command
- **sudo**: act as the super-user
- **find**, **egrep**, **cat**, **top**, **ps**, **apropos**...

<br />
<br />
<br />
<br />

And tabulation is your best friend.
