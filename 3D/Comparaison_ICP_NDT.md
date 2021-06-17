# Comparaison ICP NDT 
On constate que l'algorithme NDT donne de loin les meilleurs résultats. <Br/>
Ces programmes pcregistericp et pcregisterndt sont inchagés et utilisés tels quels. <Br/> 
Attention à utiliser une version de Matlab qui permet d'exécuter ces programmes correctement. La version R2018a les excécute bien, tandis que la version 1 de R2021 n'ecute pas les fonctions pcdownsample et pcmerge ne fonctionnent plus à cause d'une autre fonction ~~nsit'ha~~.
## ICP 
Pour un temps de calcul de 69 secondes dans la machine que j'utilise (voir hardware.md) <Br/>
On applique l'ICP Point-to-Plane étant donné que c'est l'algorithme qui donne les meilleurs résultats (voir comparaison ICPs.md) <Br/>
![ICP](https://github.com/bld-abir/SLAM/blob/75351dc3248926c984efef98bfb136e4f619e3ec/3D/3D_icp_fig_5.png)
## NDT
Pour un temps de calcul de 16 secondes <Br/>
![NDT](https://github.com/bld-abir/SLAM/blob/75351dc3248926c984efef98bfb136e4f619e3ec/3D/3D_ndt_fig_5.png)
