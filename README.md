# SLAM


## 3D Scan Matching 
On a les fichiers *pointCloudIcp.m* et *pointCloudNdt.m* qui font du **scan matching** à base de ICP et de NDT réspéctivement dans le but de recréer l'environnement de '*leavingRoom*'. On peut comparer le rendu des algo **ICP (point to point & point to plan)** et **NDT** à parir de ces deux là. 

### À faire 
1. Étudier les fonctions *pcregistericp* et *pcregisterndt*. Je crois qu'en plus de l'optimisation, il y a un proscess qui assure la **roblustesse** des résultats, on pourra décrire tout ça mɛa la **M-estimation** dans les méthodes **IRLS**.
2. Comprendre pourquoi les résultas divergents dans l'ICP koul ma je change de marge initale dans la boucle, d'autant plus que c la même chose avec le modèle 2D
3. Au lieu de se casser la tête avec le dataset el ɣaleṭ, demander à Aḥmine sa sequence 00. Et d'appliquer les mêmes commandes kima *getKitti* que je devrais importer ici. Je devrais aussi mettre en commentaire kamel les chemin en local, pour ne pas me perdre. Maintenant rani ḥafḍet'houm, mais ça ne sera plus le cas d'ici 4~5 jours
___________

## 2D Scan Matching
On a les fichiers *ScanMatchingExample_ICP.m* et *ScanMatchingExample_NDT.m* qui font idem que f le cas en 3D en 2D sur des données différentes, celles de '*scanMatchingData*'. Ce qui est cool c qu'on peut utilser le même programme pour exploiter des données issues de simulations men **ROS**. 

### À faire 
1. Chercher iḍa **Pioneer** existe fi **Gazebo**, même s'il n'est que très peu probable qu'on en vienne à exploiter un modèle qu'on générerait ḥna via ROS.
2. La **rotation** des données odométrique autour de 'Z' bi un angle 'alphao(6)' pour qu'elles correspondent l'la tranformation appliquée à la carte issue des scans.
3. Génére deux programmes pour l'icp :
   * l'un qui se contente de cartographier à partir de données odométriques, et de donées du télémetre laser, on pourra lui rajouter une optimistion grâce à l'échantionnge bi downsample. 
   * L'autre, fait l'optimisation en piquant à matchscans, ses fonctions et ses commandes. Ce qui revient à étudier matchscans en détails.
