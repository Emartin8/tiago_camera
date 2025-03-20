# ROS Workspace
# ROS2 Camera Subscriber avec Détection ArUco et Pop-up

Ce programme ROS2 abonne un noeud à un topic de caméra et détecte des marqueurs ArUco dans le flux vidéo. Lorsqu'un marqueur spécifique est détecté, il affiche des informations à propos de ce marqueur dans le flux vidéo, et dans le cas d'un marqueur particulier (ID 31), il ouvre une fenêtre pop-up pour saisir des informations.

## A quoi sert t'il?
Ce code a pour but de programmer un robot d'inventaire. Le robot scanne les codes ArUco présents sur les composants ou paquets, reportant une position pour placer le colis dans une zone donnée. Si la zone n'a pas été enregistrée auparavant, une fenêtre pop-up s'ouvre. Si celle-ci est déjà enregistrée, la zone est indiquée dans le terminal.
Après le scan de l'ArUco, le programme génère une étiquette.
Une note est également créée dans un dossier saved_notes avec toutes les informations sur le produit ainsi que la quantité.
L'image est enregistrée dans un dossier saved_images, ce qui permet, par exemple, de contrôler une notion de quantités incorrectes.

## Prérequis
1. **ROS2** (version compatible avec `rclpy`, comme ROS2 Foxy, Galactic, etc.).
2. **OpenCV** (pour la manipulation d'images et la détection de marqueurs ArUco).
3. **CvBridge** (pour la conversion entre les messages ROS et OpenCV).
4. **Tkinter** (pour la création de la fenêtre pop-up pour saisir des informations).
5. **Python 3.x**.
   
   
## Etape pour lancer le programme
   1. Ouvrir un premier terminal et lancer la commande suivante: ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=pick_and_place
   2. Gazebo Sim s'ouvre
   3. Ouvrir un autre termial et lancer la commande suivante: ros2 run tiago_camera video_stream
   4. La vue de robot se lance
   5. Ouvrir un dernier terminal et lancer la commande suivante: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel
   6. Vous pilotez à présent votre robot avec les touches I, J et L 
   7. Maintenant deplacer le programme va définir les codes aruco
   
## Détails Techniques
1. Abonnement au Topic de la Caméra
Le noeud s'abonne à un topic de caméra ROS2 (par défaut /head_front_camera/image), et utilise CvBridge pour convertir les messages d'image ROS en images utilisables par OpenCV.

2. Détection des Marqueurs ArUco
Le programme utilise OpenCV pour détecter les marqueurs ArUco dans l'image. Il utilise le dictionnaire DICT_4X4_1000 pour identifier les marqueurs dans l'image.

3. Affichage d'Informations
Lorsqu'un marqueur spécifique est détecté, un texte correspondant à ce marqueur est superposé sur l'image, et les coordonnées (X, Y) du marqueur sont également affichées à l'écran.

4. Pop-up pour la Destination
Si le marqueur avec l'ID 31 est détecté, une fenêtre pop-up s'affiche, permettant à l'utilisateur d'entrer une destination. Cette fonctionnalité est gérée dans un thread séparé pour éviter de bloquer l'exécution du programme principal.


## Fonctionnement du Code

Le code crée un noeud ROS2 qui s'abonne au topic /head_front_camera/image pour recevoir un flux vidéo depuis la caméra du robot. Ensuite, il détecte des marqueurs ArUco dans l'image et affiche des informations spécifiques basées sur les identifiants des marqueurs détectés.

   
 1. Marqueur ID 32 : Le marqueur est ignoré dans l'affichage et les coordonnées ne sont pas affichées.
 2.  Marqueur ID 124 : Affiche le texte "Moteur 380V Tri" près du marqueur dans l'image et enregistre un log.
 3.  Marqueur ID 30 : Affiche le texte "Zone moteur" près du marqueur dans l'image et enregistre un log.
 4.  Marqueur ID 40 : Affiche le texte "Zone consommable" près du marqueur dans l'image et enregistre un log.
 5.  Marqueur ID 50 : Affiche le texte "Zone électrique" près du marqueur dans l'image et enregistre un log.
 6.  Marqueur ID 31 : Ouvre une fenêtre pop-up où l'utilisateur peut entrer une "destination". Cette pop-up n'apparaît que si elle n'a pas été ouverte précédemment pour éviter qu'elle ne se répète.

Le programme utilise également un thread séparé pour afficher la fenêtre pop-up, ce qui permet à la détection des marqueurs de continuer sans être bloquée par la fenêtre graphique.




### Installation des dépendances

Assure-toi que tu as ROS2 installé et que ton environnement de travail est correctement configuré. Ensuite, installe les bibliothèques nécessaires avec les commandes suivantes :

```bash
sudo apt update
sudo apt install ros-<distro>-cv-bridge python3-opencv python3-tk





