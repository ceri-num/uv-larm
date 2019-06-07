UV LARM 1 - mini-projet

#Déplacer un robot mobile

L'objectif de ce TP est de réaliser un noeud ROS qui prend en paramètre une position à atteindre et qui contrôle les vitesses du robot pour lui permettre d'atteindre cette position cible.

Notion :
  * Creer des noeuds paramétrable.
  * Gérer correctement les transformations.

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
