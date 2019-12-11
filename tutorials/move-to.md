# 02 - Move a robot

This tutorial aims to take control over turtlebut2 kobuki platform helped by laserange to nagivate in a cluterred environement.

## Installation:

Targeted robot is a TurttleBot2 (Kobuki version)

First you have to install turtlebut2 packages that allow you to interact with the robot throught ROS topics.

Instructions for ROS melodic version are availlable on github: [github.com/gaunthan/Turtlebot2-On-Melodic](https://github.com/gaunthan/Turtlebot2-On-Melodic)



```bash
sudo apt
```

Then, the connextion to the robot is performed trought USB cable.

```bash
dialout
```

#### Draft:

L'objectif de ce TP est de réaliser un noeud ROS qui prend en paramètre une position à atteindre et qui contrôle les vitesses du robot pour lui permettre d'atteindre cette position cible.

Notion :
  * Interpréter des données laser
  * Gérer des commande de déplacement
  * Prendre la main sur les outils générique de ROS
    - les commandes ros (rostopic ...)
    - rviz
    - rqt_graph
    - les bagfiles

## Bouger une base mobile

Architecture, topic et commande

  * Un launch file
  * rqt_graph / view_frame ...
  * faire un noeud move-to qui publie des geometry-twist au bon endroit

## Gerer une position cible

  * Se connecter à odom.
  * Déplacer le robot de façon à ce que la position cible défini dans odom soit atteint (transformation base/odom).

## Actualiser la position cible

  * Creer un topic Goal et actualiser la position cible (transformation goal_frame/odom).
