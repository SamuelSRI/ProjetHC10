# ProjetHC10

## 📌 Découpage du projet en tâches
1. Génération de trajectoires (MoveIt + Python)

T1.1 Lire et comprendre l’API Python MoveGroupCommander (MoveIt).

T1.2 Écrire un script Python simple qui déplace l’outil du robot d’un point A à un point B.

T1.3 Étendre le script pour générer une trajectoire cartésienne (suivre une ligne, un carré, etc.).

T1.4 Tester et documenter (screenshots RViz).

🔗 Dépend de : robot fonctionnel dans MoveIt (déjà fait).

2. Simulation Gazebo

T2.1 Mettre à jour le fichier hc10_macro.xacro pour inclure les plugins Gazebo (gazebo_ros_control, inerties, joints).

T2.2 Vérifier le lancement avec demo_gazebo.launch.

T2.3 Comparer le comportement RViz (cinématique) vs Gazebo (cinématique + physique).

T2.4 Documentation + capture d’écran.

🔗 Dépend de : modèle URDF/XACRO complet du HC10.

3. Intégration du capteur 3D (ex: Kinect)

T3.1 Récupérer un modèle de capteur (déjà fourni par ROS, ex. kinect.urdf.xacro).

T3.2 Positionner le capteur par rapport au robot (ajout dans hc10_macro.xacro).

T3.3 Vérifier que Gazebo publie bien des données pointcloud.

T3.4 Documentation + schéma du montage.

🔗 Dépend de : Gazebo fonctionnel (T2).

4. Couplage perception + MoveIt

T4.1 Mettre en place le perception pipeline de MoveIt (moveit_sensor_manager, occupancy_map_monitor).

T4.2 Vérifier que le nuage de points du capteur est bien utilisé par MoveIt pour éviter les collisions.

T4.3 Tester : placer un obstacle devant le robot → MoveIt doit recalculer une trajectoire évitant l’obstacle.

T4.4 Documentation + vidéo/animation.

🔗 Dépend de : capteur intégré (T3).

5. Gestion du dépôt GitHub/GitLab

T5.1 Créer un dépôt clair avec arborescence (src/hc10_moveit_config, src/hc10_sim, scripts/).

T5.2 Rédiger un README clair (installation, lancement, scripts).

T5.3 Vérifier que tout est reproductible depuis zéro (VM propre).

🔗 Parallèle aux autres tâches, mais doit être validé en fin de projet.

6. Rapport écrit

T6.1 Introduction + objectifs.

T6.2 Méthodologie (ROS, MoveIt, Gazebo, capteur).

T6.3 Résultats (trajectoires, simulation, perception).

T6.4 Conclusion + pistes futures.

🔗 Dépend de toutes les parties techniques.

7. Présentation orale

T7.1 Préparer slides (intro, méthode, démos, résultats, conclusion).

T7.2 Répartition des interventions entre les 10.

T7.3 Répétition → respecter le temps.

🔗 Dépend de : T6 (rapport).
