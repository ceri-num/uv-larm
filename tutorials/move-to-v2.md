# 04 - git Déplacer un robot mobile V2

L'objectif de ce TP est de réaliser un noeud ROS qui prend en paramètre une position à atteindre et qui contrôle les vitesses du robot pour lui permettre d'atteindre cette position cible.

Notion :
  * Creer une architecture de plusieurs noeuds
  * rendre les noeuds paramétrable.
  * Gérer correctement les transformations.
  * (Voire mertre en lace des services..?)

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