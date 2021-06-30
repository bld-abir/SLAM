# Point Cloud 

Le \textit{pointcloud} est une structure de base pour décrire une image constituée d'une densité certaine de points. Le \textit{pointcloud} dispose d'une structure matricielle recellant l'intégraité des données concernant chaque point du nuage de point en question 


# M-Estimateur

L'une des techniques populaire qui ont pour but d'assurer la robustesse est celle des M-Estimateurs. L

# ICP Désavantages 
L'ICP et ses variantes sont des techniques rigides de scan matching de nuages de points, utilisées dans un large éventail d'applications de la robotique à la reconstitution 3D. Les obstacles phares auxquels cet algorithme est confronté est la lenteur de la convergence et la sensibilité aux outliers jusqu'à l'passer sous silence de données valides, et le chevauchement partiel. 

# IRLS 

Le scan matching rigid qui trouve une transformation rigide optimale qui alligne un nuage de point source est le problèle fondamental en \textit{Computer Vision} de même que plusieurs autres domaines de recherche. L'algorithme de l'ICP est une méthode classique de scan matching rigid. Il alterne entre la question de point le plus proche visé et minimisation de distance entre points correspondants, il garanti la convergence à un alignement localement optimal. Cela dit l'ICP classique souffre de la lenteur de la convergence, une lenteur atténuée dans certaines variantes comme l'alignement réalisé via minimisation des distances point à plan (voir section ....) , ou encore en minisant l'approximation quadratique locale de la fonction de la distance carée\cite{minQuadr}. D'autres invénients du ICP demeurent tel que la précision de l'alignement qui peut être afféctéée par les imperfections du nuage de point tels que les bruits, les outliers, les chevauchements partiels, des imperfections très récurrentes dans des modèles d'état et de mesure réels. Diverses techniques ont vu le jour justement pour palier à cette problématiqu et c'est justement cette problématique que traitera cette variante, ou plutôt cette famille de variantes, puisque l'IRLS-ICP regroupe plusieurs méthodes que nous détaillerons plus bas.

Une méthode heuristique populaire existe dans l'optique de négliger les correspondances érronées, elle se base sur la distance ou l'angle de leurs normales\cite{heuristic}. Puis une approche de minimisation de norme $ l_p $ a été proposée dans \cite