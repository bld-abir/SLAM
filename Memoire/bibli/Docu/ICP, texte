ICP, stands for Iterative Closest Point. It is a technique used in order to align to point out with each other. ICP is a key building block for laser-based SLAM

So waht makes point cloud alignment ?

The Point cloud is -as mentionnend before- a set of 2D or 3 points. 
What we wanna do, is to compute/estimate the transfromation to move one cloud so that it is aligned with the other one.
And the goal is that through this transformation points that are the same points that are the same point in the real world, actually lie on top of each other, so that we can incremetly build a onsistentn model of the enviromnment.

And therefore that's a key building block of most slam algorthm for exmaple those that work based on laser rangefiders (LADAR) or RGBD cameras

ICP is used in SLAM systems for LIAR or RGBD data 
There are several ICP variants.

So, How does it work ?
The Vanilla ICP algorithm is the basic form of the Itirative Closest Point algorithm.
It has basically two steps :
Data Association & computing the Trnsformation based on the Data assocaition
Exemple :
Image 1
We want to align the blue and the red point clouds. We can do this by using a simple earest neghbour approach, that means that we iterate over one point cloud an look for th closest point to these points in the second point dot and then make this alignment and these are the correspondences of data association that they have resulted in based on this initial configuration 
image 2 avec le fils 
ANdthe second step takes this data assocaition and tries to fond the transformation so that those two points are moved on top of eachother to minimize the distnces between point pairs. And this goes in two steps :

1ST Step : Compute centers of mass (image)
2nd Step : Compute the optimal rotation using an SVD basd approch so that the points are better aligned.

Remaque : Estimating the data association is the key challenge :
We say better and not perfectly allignd bcause the inter data association may not have been perfect. Tat's why we repeat thisprocess, based on the gueses that we ave in a first itiration that the points are alligned, and we recompute the data association and thn recompute the alignment, and we do this iterativly : Data Association & Alignment, until we coverge 

Image 

Other ICP algorithms :
They aim to optimise the ICP algorithm. One prominent exemple is the Point to Plane metric. It is an approche we are going to use in the experimentation.

We start by a simple hypothisis, according to whisch we assume that the points don't stem, or that they are not distinct points in the environment, but that they actually stem from a surface. They are basically randomly centered trouh the scanner from a surface, and by making an alignment between the point cloud and the surface, we don't create an artificial point-to-point alignment whihch may not exist in the real world.
This point of plane alignment typiclly or put a plane metric to really givesa better result of the ICP algorithm but it does require t leave the SVD based approcha and use instead the general least squares approach, then we would most liely use the Gauss Newtom method in order to minimise the error under this point to play metric.

Projective ICP 
We take the model of Point Cloud that we dispose pf and project it into the range image creted by the scanner, and then try to find the alignment.

Robust Kernels  
Is an a method used in the error minimisation of promotion oulier's promotion. So that if we have outliers situatio where some of the points are really wrongly aligned, the those outliers ejection techniues help in order to weigh thse points down and ignore them in the data association, or reduce their impact when computing the transformation.

Thre are several variants of ICP, ,and the choice may depend on the problem that we are adressing to consider which of the variants we would take more avantage of.

Point-to-palne ICP is today the standared for aligning point clouds, thosethat stem from a laser range scanner.
There areseveral other variants 