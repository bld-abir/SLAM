# SLAM 2d

## Scan Matching de 2 Scans :
Lorsqu'on utilise l'algorithme de l'ICP sans aucune optimisation, rien qu'en utilisant les moindres carrés, on obtient un très mauvais résultat, comparés aux résultats qu'on retrouve avec la NDT.<Br/>
Par contre, en optimisant la robustesse de l'algorithme de l'ICP, que ce soit avec la M-Estimation de Cauchy ou Weiner, on retrouve avec l'ICP de meilleurs résultats que via la NDT.

## SLAM pour localiser et cartographier un environnement inconnu :
La fonction matlab **matchscans** pour appliquer la NDT, est parfaite pour la cartographie et la localisation, elle est parvenue à recontruire l'environement presque sans faute, les résultats sont parfaitement utilisable pour la navigation.
Celà dit, les résultats obtenus uniquement via le scan matching dans le cas du programme de l'ICP manque drastiquement de correction. On remarque donc que le résultat est loin d'ếtre satisfaisant, surtout comparé à celui de la NDT.
