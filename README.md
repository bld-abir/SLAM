# SLAM


## 3D Scan Matching 
On a les fichiers *pointCloudIcp.m* et *pointCloudNdt.m* qui font du **scan matching** à base de ICP et de NDT réspéctivement dans le but de recréer l'environnement de '*leavingRoom*'. On peut comparer le rendu des algo **ICP (point to point & point to plan)** et **NDT** à parir de ces deux là. 
On s'est basé sur : [Matlab 3-D Point Cloud Registration and Stitching](https://fr.mathworks.com/help/vision/ug/3-d-point-cloud-registration-and-stitching.html) et de [Matlab Example pcregisterndt](https://fr.mathworks.com/help/vision/ref/pcregisterndt.html)</Br>
Les fonctions *pcregistericp* et *pcregisterndt* suivent le modèle SLAM Overall (décrit fi le chapitre 1er fi la section nsit) le scan matching qui représente la première étape de ce SLAM (front end) puis l'étape de back end est assurée par la méthode d'optimisation Pose Graph.

### À faire 
1. [ ] Retélécharger kitti, cette fois ça prendra le temps que ça prendra...
2. [ ] Appliquer le code l'une des séquences (03 ?) sur le dataset Kitti l'ICP et la NDT. Normalement ça sera facile de lire les donnés si je m'en souviens encore. Et comparer avec groundtruth.
3. [ ] Dessiner la trajectoire du robot pour que ce ne soit pas de la construction 3D, mais du SLAM.

___________

## 2D Scan Matching
On a les fichiers *ScanMatchingExample_ICP.m* et *ScanMatchingExample_NDT.m* qui font idem que f le cas en 3D en 2D sur des données différentes, celles de '*scanMatchingData*'. Ce qui est cool c qu'on peut utilser le même programme pour exploiter des données issues de simulations men **ROS**. </Br>
L'ICP-IRLS se base sur l'ICP Vanilla mais en rajoutant un processus d'optimisation dans la (2ème wella 3ème étape), un proscess qui assure la **roblustesse** de l'algorithme, des méthodes de **M-estimation**.</Br>
On n'a rien à tirer de *matchscans*. J'y ai deja passé assez de temps. Balak je revérifie iḍa ils usilisent PoseGraph.

### À faire 
[x] La **rotation** des données odométrique autour de 'Z' bi un angle 'alphao(6)' pour qu'elles correspondent l'la tranformation appliquée à la carte issue des scans.
[ ] Génére deux programmes pour l'icp :
   * [x] l'un qui se contente de cartographier à partir de données odométriques, et de donées du télémetre laser, on pourra lui rajouter une optimistion grâce à l'échantionnge bi downsample. 
   * [ ] L'autre, fait l'optimisation avec PoseGraph, et on concerve les données du télémètre laser même pour les poses. 
[ ] Essayer d'autres ICP en ajoutant à nos scans 2D une colonne Z de zéros comme 1ère solution (pour avoir des scans 3D) et voir est-ce que ça marche.
   * [ ] la 2ème fonction icp 3D 
   * [ ] et celle de Matlab
___________

## Mémoire 

### À faire 
[ ] Mettre en commentaire kamel les chemins en local, pour ne pas me perdre. </Br>
[x] Récuperer les images men un ancien document pdf ;</Br>
[ ] Trouver une solution pour récuperer le maximum de references possibles. Kayen deja :</Br>
    1. Ceux dont j'ai concervé les liens ;</Br>
    2. Ceux que j'ai mis bekri sur Drive ;</Br>
    3. Ceux que j'ai utilisé fel memoire et qui sont referencés f'la bibliothèque</Br>
[ ] Rejection des points :
    1. 3_LV_Jixin / p.22 Rejection (ma fhamtch wesh ktebt) ~~ Distance~~ Des traces~~
    2. 12_FastIcp / p.5 Refection Pairs
[ ] Overall Slam : 11_Nahman
[ ] IRLS :
    1. 13_FastAndRobust / p.5 Robust Icp
    2. 14_Robust / Intro & M-Estimation
    3. 15_Comparaison_Cauchy_Welsch_Huber / p.1 FigI ; p.2 III Theory ; p5 Comparaison with l2
[ ] Format: revoir la mise en page (les marges): Chercher ce qu'est le format exigé par le département, c bon pour la taille (12) et la police (Times New Roman), reste les interlignes (1.5), les marges et autres ?</Br>
[ ] Justifier les détails sur EKF et filtre Particulaire, sinon supprimer ;</Br>
[ ] Mettre des leins de renvoi partout où il y a label : figure, tableaux, références bibliographiques ;</Br>
[ ] Réécrire la section 3.4 Défits courants, ou la supprimer ;</Br>
[ ] Mettre en avant l'introduction du chapitre 1 avec un titre dédié ;</Br>

___________

## Simulations 

### À faire 
[ ] Retélécharger : [ ] Matlab R2018 Ver.Linux ; [ ] PyCharm ; [ ] Ros (je me suis mise à l'aise mɛa la version Melodic, voir si Bionic temchi mɛa Matlab 2018) ; </Br>
[ ] Toutes les fonctions fi la toolbox ~~ *Robotics/.../examplalgs* ~~ mchawli même ceux du FP w le FK, ça serait bien d'essayer de les appliquer ɛla la bdd exploitee dacs le cas 2D wella 3D</Br>

___________

## In an other life...
1. [ ] Chercher iḍa **Pioneer** existe fi **Gazebo**, même s'il n'est que très peu probable qu'on en vienne à exploiter un modèle qu'on générerait ḥna via ROS.

___________

## À rediscuter :
1. Pour chaque paire de scan, faire un calcul d'erreur (res de icp2 par exemple) si res<seuil on prend en considération la pose relative obtenue Transform, sinon on ne considère pas la Transform obtenue et on passe au scan suivant. </Br>
=> C déja le cas, l'IRLS utilise des méthodes de rejection des points concidrés comme étant invalides... Le seuillage se fait via 
~~~
345 |   if abs(oldres-res) < thres
346 |         break
~~~
tel que *thres* c le seuil de rejection, et *res* et *oldres* sont les la contrainte calculée dans le cas du critère 0 ainsi
~~~
229 | res=mean(resid.^2);
~~~
ou dans le cas des critères 1, 2, 3 ou 4 (M-Estimateurs)
~~~
257 |  kRob = 1.9*median(resid);
258 |          
258 |            maxResid=max(resid);
260 |            if kRob<1e-6*maxResid
261 |                kRob=0.3*maxResid;
262 |            elseif maxResid==0
263 |                kRob=1;
264 |            end
265 |            
266 |            res=mean(resid(resid<1.5*kRob).^2);
~~~
tel que resid est soit la norme euclidiéne de la distance entre les deux nuages de points, calculée soit via *norm* directement ou via *nearestNeighbor* (qui a besoin d'une représenation trigulaire, ici on utilise [*delaunayTriangulation* du nuage de point "modèle"] avec [le nuage de point "data"])
![norleEuclid](https://user-images.githubusercontent.com/53100788/124046218-56c14680-da09-11eb-836f-580696a78550.png)
![nearestNeighbor](https://user-images.githubusercontent.com/53100788/124046944-03e88e80-da0b-11eb-8995-ba7ab54dc585.png)

2. Chercher après ICP (Martin Kjer & Jakob Wilm) et rédiger dessus une secion dans variantes, il est détaillée dans la thèse bscthesis.pdf</Br>
=> Ce n'est pas un algorithme, il parle en général des divrses possibilités pour chacune des étapes de l'ICP et il les compare.</Br>

3. les deux variantes ICP2 (de  Bergström)</Br>
=> C'est IRLS-ICP, dans ce papier, il parle de 3 variantes, Tukey bi-weighted, Cauchy et Huber. Il en existe bien d'autres, et justement dans l'icp2, on utilise l'algorithme L2, Tukey bi-weighted, Cauchy, Huber et Welsh. </Br>

