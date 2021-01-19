# Évaluation intermédiaire

### Groupe évaluateur : groupe X

Membres : (prénom nom)

 - > 
 - >
 - >

## Fiche d'identité du projet

- Groupe évalué : **groupe Y**
- Dépôt git : [link](https:url_github)
- numéro du commit (8 premiers caractères) : **8665ebe7**
- Date du commit : **Jan 13 22:33:14 2042**

cf. `git log`


## Grille d'évaluation générale

Chaque question est évaluée sur une échelle de -1 à 4:

- **-1** : Pire que rien, rien aurait été mieux, dans la mesure ou le peut présenté ne fait que témoigner de la non-compréhension du groupe par rapport à ce qui peut être attendu.
- **0**  : Pas d'éléments, ou les éléments présents sont proche de rien (simple initialisation par exemple de ressources/fichiers pour l'instant vide).
- **1** : Ni fait Ni à faire, il y a des éléments qui vont dans le bon sens, mais trop peut ou erroné.
- **2** : Incomplet, les éléments répondent aux attendes dans l'ensemble, mais avec des lacunes sur certains points.
- **3** : Ok, les éléments répondent aux attentes.
- **4** : Impecable, les éléments répondent aux attentes et vont au-delà du strict nécessaire.

En résumé : 

-1            |     0          |   1   |    2      |   3  |    4
--------------|----------------|-------|-----------|------|----------
Pire que rien | Pas d'éléments | Ni-ni | Incomplet |  Ok  | Impeccable


## Installation

Concerne l'appropriation de la solution par les évaluateurs de façons construire une solution prête à être évaluée et la documentation liée à cette action (notamment dans `README.md`).

1. La page d'accueil est propre et renseignée sur le projet ? 
   - Évaluation : **X**
   - Commentaire :
2. Les consignes d'installation sont faciles à trouver ?
   - Évaluation : **X**
   - Commentaire :
4. La procédure d'appropriation de la solution s'effectue sans encombre (clone/téléchargement ...) ?
   - Évaluation : **X**
   - Commentaire :
5. La Construction de la solution s'effectue correctement (compilation ...) ?
   - Évaluation : **X**
   - Commentaire :
6. Le protocole d'installation est clair et documenté ?
   - Évaluation : **X**
   - Commentaire :

## Challenge 1

Le robot se déplace dans un environnement encombré.

1. Le lancement du challenge s'effectue correctement (launch des `challenge-1.launch` et `navigation.launch`) ? 
   - Évaluation : **X**
   - Commentaire :
2. Les consignes qu'en aux opérations à effectué par l'opérateur sont claires, cohérentes et minimales ?
   - Évaluation : **X**
   - Commentaire :
1. La réalisation des opérations s'effectue correctement ?
   - Évaluation : **X**
   - Commentaire :
2. Le robot se déplace correctement vers une position `goal` en évitant les obstacles ?
   - Evaluation : **X**
   - Commentaire :
3. Les trajectoires suivies par le robot sont souples et efficaces ?
   - Évaluation : **X**
   - Commentaire :
4. (BONUS) Le robot peut se déplacer en dehors de ça carte ou n'utilise pas de carte ?
   - Évaluation : **X**
   - Commentaire :
5. Conclusion générale sur le challenge 1 ?
   - Évaluation : **X**
   - Commentaire :

## Challenge 2

Le robot se cartographie et trouve les bouteilles dans un environnement de type intérieur/bureau.

1. Le lancement du challenge s'effectue correctement (sur la base des launchs appropriés) ? 
   - Évaluation : **X**
   - Commentaire :
2. Les consignes qu'en aux opérations à effectué par l'opérateur sont claires, cohérentes et minimales ?
   - Évaluation : **X**
   - Commentaire :
3. La réalisation des opérations s'effectue correctement ?
   - Évaluation : **X**
   - Commentaire :
4. Le robot construit une carte au fils de ses déplacements ?
   - Évaluation : **X**
   - Commentaire :
5. Le robot est efficace pour détecter une bouteille lorsqu'elle est devant lui ? 
   - Évaluation : **X**
   - Commentaire :
1. La position de la bouteille et remonté dans un topic approprié et visualisé dans `rviz` ?
   - Évaluation : **X**
   - Commentaire :
2. Le robot trouve toutes les bouteilles et sans faux positifs ?
   - Évaluation : **X**
   - Commentaire :
3. (BONUS) La position de chaque bouteille n'est envoyée qu'une seule foi (même si le robot repasse une seconde fois) et l'affichage dans `rviz` est persistant ?
   - Évaluation : **X**
   - Commentaire :
4.  Les sorties de la solution sont de bonne qualité (carte, position des objets dans la carte) ?
   - Évaluation : **X**
   - Commentaire : 
1. Conclusion générale sur le challenge 2 ?
   - Évaluation : **X**
   - Commentaire : 

## Challendge 3

Le robot explore de façons autonomes un environnement de type intérieur/bureau.

1. Le lancement du challenge s'effectue correctement (sur la base des launchs appropriés) ? 
   - Évaluation : **X**
   - Commentaire :
2. Les opérations à effectuer par l'opérateur sont proches du néant ?
   - Évaluation : **X**
   - Commentaire :
3. Le robot se déplace efficacement de façons autonomes ?
   - Évaluation : **X**
   - Commentaire :
4. La stratégie d'exploration (succession de position à atteindre) semble efficace ?
   - Évaluation : **X**
   - Commentaire :
5. Le robot détecte et communique la position des bouteilles quand il en croise une ?
   - Évaluation : **X**
   - Commentaire :
6.  Les sorties de la solution sont de bonne qualité (carte, position des abjects dans la carte) ?
   - Évaluation : **X**
   - Commentaire : 
5. (BONUS) Le robot stoppe automatiquement lorsque sa carte est complète et le signale ?
   - Évaluation : **X**
   - Commentaire :
6. Conclusion générale sur le challenge 3 ?
   - Évaluation : **X**
   - Commentaire :

## Le dépôt

On s'intéresse ici aux fichiers rendus via le dépôt en rentrant un peu plus dans les sources.
Attention, on évalue uniquement la branche principale, et le commit sus-mentionné.

1. Le dépôt est propre, les fichiers sont rangés dans les répertoires appropriés selon les préconisations ROS ? 
   - Évaluation : **X**
   - Commentaire :
2. Le dépôt est minimal, il ne contient que les fichiers sources, utile à la solution ? 
   - Évaluation : **X**
   - Commentaire :
3. Les launch files et les fichiers de paramètres sont bien nommés, clairs et concis ... ? 
   - Évaluation : **X**
   - Commentaire :
4. Les sources python ou Cpp sont propres (on peut ouvrir les sources et s'y retrouver) ?
   - Évaluation : **X**
   - Commentaire :
5. (si question précédente est à minima **2-incomplete**) les sources s'appuient sur des structures ou des classes et des fonctions unitaires quand il y a lieu de le faire ? 
   - Évaluation : **X**
   - Commentaire :
6. (si question précédente est à minima **2-incomplete**) les algorithmes mis en oeuvre 'semble' efficaces (pas de boucles inutiles, ou de possible optimisation évidente) ? 
   - Évaluation : **X**
   - Commentaire :

## Avis global sur la solution

Évaluation : **X** ( on devrait être ici **2-incomplet** )
Commentaire : (fait état des challenges qui peuvent être conservés validé et de ceux encore à faire et surtout fait la différence entre une solution ou ce qui est fait est propre et une solution ou tout ou partie de ce qui est fait est à revoir. Idéalement donc, seuls quelques points évalués **1** ou **2** l'ont été, car le travail est inachevé, autrement on rencontre essentiellement des **0** (point non encore adressé) **3** ou **4**.)
Avis : (est-ce qu'il vous semble compliqué pour le groupe évalué de finir le travail d'ici à vendredi ?)
Avis miroir :  (est-ce qu'il vous semble compliqué pour vous (le groupe évaluateur) de finir le travail d'ici à vendredi ?)
