# SLAM


## 3D Scan Matching 
On a les fichiers *pointCloudIcp.m* et *pointCloudNdt.m* qui font du **scan matching** à base de ICP et de NDT réspéctivement dans le but de recréer l'environnement de '*leavingRoom*'. On peut comparer le rendu des algo **ICP (point to point & point to plan)** et **NDT** à parir de ces deux là. 
On s'est basé sur : [Matlab 3-D Point Cloud Registration and Stitching](https://fr.mathworks.com/help/vision/ug/3-d-point-cloud-registration-and-stitching.html) et de [Matlab Example pcregisterndt](https://fr.mathworks.com/help/vision/ref/pcregisterndt.html)
Les fonctions *pcregistericp* et *pcregisterndt* suivent le modèle SLAM Overall (décrit fi le chapitre 1er fi la section nsit) le scan matching qui représente la première étape de ce SLAM (front end) puis l'étape de back end est assurée par la méthode d'optimisation Pose Graph.

### À faire 
1. [ ] Retélécharger kitti, cette fois ça prendra le temps que ça prendra...
2. [ ] Appliquer sur le dataset Kitti l'ICP et la NDT. Normalement ça sera facile de lire les donnés si je m'en souviens encore. 
3. [ ] Dessiner la trajectoire du robot

___________

## 2D Scan Matching
On a les fichiers *ScanMatchingExample_ICP.m* et *ScanMatchingExample_NDT.m* qui font idem que f le cas en 3D en 2D sur des données différentes, celles de '*scanMatchingData*'. Ce qui est cool c qu'on peut utilser le même programme pour exploiter des données issues de simulations men **ROS**. 
L'ICP-IRLS se base sur l'ICP Vanilla mais en rajoutant un processus d'optimisation dans la (2ème wella 3ème étape), un proscess qui assure la **roblustesse** de l'algorithme, des méthodes de **M-estimation**.
On n'a rien à tirer de *matchscans*. J'y ai deja passé assez de temps. Balak je revérifie iḍa ils usilisent PoseGraph.

### À faire 
[X] La **rotation** des données odométrique autour de 'Z' bi un angle 'alphao(6)' pour qu'elles correspondent l'la tranformation appliquée à la carte issue des scans.
[ ] Génére deux programmes pour l'icp :
   * [x] l'un qui se contente de cartographier à partir de données odométriques, et de donées du télémetre laser, on pourra lui rajouter une optimistion grâce à l'échantionnge bi downsample. 
   * [ ] L'autre, fait l'optimisation avec PoseGraph, et on concerve les données du télémètre laser même pour les poses. 

___________

## Mémoire 

### À faire 
[ ] Mettre en commentaire kamel les chemins en local, pour ne pas me perdre. 
[ ] Récuperer les images men un ancien document pdf ;
[ ] Trouver une solution pour récuperer le maximum de references possibles. Kayen deja :
    1. Ceux dont j'ai concervé les liens ;
    2. Ceux que j'ai mis bekri sur Drive ;
    3. Ceux que j'ai utilisé fel memoire et qui sont referencés f'la bibliothèque

___________

## Simulations 

### À faire 
[ ] Retélécharger : [] Matlab R2018 Ver.Linux ; PyCharm ; Ros (je me suis mise à l'aise mɛa la version Melodic, voir si Bionic temchi mɛa Matlab 2018) ; 
[ ] Toutes les fonctions fi la toolbox ~~ *Robotics/.../examplalgs* ~~ mchawli même ceux du filtre particulaire w le FK, ça serait bien d'essayer de les appliquer ɛla la bdd exploitee dacs le cas 2D wella 3D

___________
## In an other life...
1. [ ] Chercher iḍa **Pioneer** existe fi **Gazebo**, même s'il n'est que très peu probable qu'on en vienne à exploiter un modèle qu'on générerait ḥna via ROS.
