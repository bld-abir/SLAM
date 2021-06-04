# SLAM


## 3D Scan Matching 
On a les fichiers pointCloudIcp.m et pointCloudNdt.m qui font du scan matching à base de ICP et de NDT réspéctivement dans le but de recréer l'environnement de 'leavingRoom'. On peut comparer le rendu des algo ICP (point to point & point to plan) et NDT à parir de ces deux là. 

### À faire 
1. Étudier les fonctions pcregistericp et pcregisterndt. Je crois qu'en plus de l'optimisation, il y a un proscess qui assure la roblustesse des résultats, on pourra décrire tout ça m3a la M-estimation dans les méthodes IRLS.
2. Comprendre pourquoi les résultas divergents dans l'ICP koul ma je change de marge initale dans la boucle, d'autant plus que c la même chose avec le modèle 2D
___________
## 2D Scan Matching
