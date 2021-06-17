# Comparaison ICP_Pl e t ICP_Pt

La différence commence lors du calcule du Jacobien pour appliquer la méthode des moindres carés.<br/>
(On applique cette méthode pour un système non linéaire, et donc la méthode d Gauss Newton, 
puisque le problème qu'on cherche à minimiser contient une matrice de rotation)<br/>
Après avoir obtenu la matrice R de rotation et le vecteur T de translation, via la décomposition SVD, on applique Gauss Newton, et alors on calcule le Jacobien.<br/>

### Dans le cas de l'ICP_Pt
On aura une matrice [2x4] <br/>
```
J=(I,Rg*p) 
```
*J étant le Jacobien ;* <br/>
_I :  l'identité ;_<br/>
*R : La matrice de Rotation obtenue après la minimisation ;*<br/>
*p(i) : le point appartenant au currentScan qu'on veut matcher c un vecteur [p(i)x p(i)y]t)*.
![icp po_to_pt](https://github.com/bld-abir/SLAM/blob/main/3D/3D_icp_fig_5_po_to_pt.png?raw=true)

### Dans le cas de l'ICP_Pl
On aura une matrice [1x3] <br/>
L'erreur à minimiser sera différente. Au lieu de minimiser l'erreur euclédienne entre 2 points (p(i) et q(i)) ou plutôt 2 uages de points (P(i) et Q(i)), on va plutôt minimiser l'erreur projetéee sur "the normal vector" de l'un des points. <br/>
```
J=[nx ny nx(Rg(1,:)*p)+ny(Rg(2,:)*p)]
```
_ici n c'est n(i) qui est la "normal direction" du point q(i)._ <br/>
![icp po_to_pl](https://github.com/bld-abir/SLAM/blob/main/3D/3D_icp_fig_5_po_to_pl.png?raw=true)

## Le résultat :
L'ICP_Pt est nettement moins efficace que l'ICP_Pl <br/>

**Pl. : Point-to-Plane**<br/>
**Pt. : Point-to-Point**
