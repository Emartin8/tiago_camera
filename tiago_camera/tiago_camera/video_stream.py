import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import tkinter as tk
from tkinter import simpledialog
import os  # Pour la gestion de fichiers

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image',  # Remplacer par topic robot
            self.listener_callback,
            10  # Taille de la file d'attente du message
        )

        # Initialisation CvBridge pour convertir les messages ROS OpenCV
        self.bridge = CvBridge()

        # Paramètres ArUco
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)  # Type de dictionnaire ArUco
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Variable de contrôle d'état du pop-up
        self.popup_opened = False
        self.destination_set = False  #variable pour savoir si la destination a été définie
        self.destination = ""  # Pour stocker la destination

        # Dossier pour enregistrer les notes
        self.save_dir = "saved_notes"
        if not os.path.exists(self.save_dir):  # Créer le dossier s'il n'existe pas
            os.makedirs(self.save_dir)

    def open_popup(self):
        # ouvrir la fenêtre pop-up dans une nouvelle fenêtre
        def popup_thread():
            root = tk.Tk()
            root.withdraw()  # Cacher la fenêtre principale

            # Affichage de la pop-up dans une fenêtre
            self.destination = simpledialog.askstring("Imformations produit", "ID, position, quantité, destination:")
            if self.destination:
                self.get_logger().info(f"Imfomations produit reçue : {self.destination}")
                self.destination_set = True  # Marquer la destination comme définie

            self.popup_opened = False  # Réinitialiser pour permettre d'ouvrir à nouveau la pop-up
            root.quit()  # Fermer la fenêtre Tkinter correctement
            root.destroy()  # Nettoyer la fenêtre Tkinter

        # Lancer la fenêtre pop-up dans un thread pour ne pas bloquer l'exécution
        if not self.popup_opened and not self.destination_set:  # Ouvrir le pop-up seulement si non ouvert et destination non définie
            self.popup_opened = True
            threading.Thread(target=popup_thread, daemon=True).start()

    def generate_note(self, object_name, aruco_id, position, quantity):
        # Générer la note avec les infomations
        note = f"Nom: {object_name}\nID: {aruco_id}\nPosition: ({position[0]:.2f}, {position[1]:.2f})\nQuantité: {quantity}\nDestination: {self.destination}"
        return note

        #Détection des Aruco partie 2
    def listener_callback(self, msg):
        try:
            # Convertir le message d'image ROS en image OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Détecter les marqueurs ArUco
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_parameters)

            if ids is not None:
                # Affichage des marqueurs détectés sur l'écran
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

                for i in range(len(ids)):
                    c = corners[i][0]
                    center_x = np.mean(c[:, 0])
                    center_y = np.mean(c[:, 1])

                    # Ignorer l'affichage de l'ArUco avec ID 32
                    if ids[i][0] == 32:
                        continue  

                    object_name = ""
                    quantity = 1  # Init=une seule unité détectée

                    # Si l'ID=24, afficher l'information du produit
                    if ids[i][0] == 124:
                        self.get_logger().info("ID 124 détecté : Ce marqueur correspond à un moteur tri 380V.")
                        object_name = 'Moteur 380V Tri'
                    # Si l'ID=30, afficher l'information du produit
                    elif ids[i][0] == 30:
                        self.get_logger().info("ID 30 détecté : Ce marqueur correspond à une zone moteur.")
                        object_name = 'Zone moteur'
                    # Si l'ID=40, afficher l'information du produit
                    elif ids[i][0] == 40:
                        self.get_logger().info("ID 40 détecté : Ce marqueur correspond à une zone consommable.")
                        object_name = 'Zone consommable'
                    # Si l'ID=50, afficher l'information du produit
                    elif ids[i][0] == 50:
                        self.get_logger().info("ID 50 détecté : Ce marqueur correspond à une zone électrique.")
                        object_name = 'Zone électrique'
                    
                    # Si l'ID=31, ouvrir la pop-up pour la destination
                    if ids[i][0] == 31 and not self.popup_opened:
                        self.open_popup()  # Appeler la méthode pour ouvrir la fenêtre pop-up

                    # Si destination=true et que l'ID correspond, générer une note
                    if self.destination_set and object_name:
                        note = self.generate_note(object_name, ids[i][0], (center_x, center_y), quantity)

                        # Envoie de la Sauvegarde dans un fichier texte
                        note_filename = f"note_{ids[i][0]}.txt"
                        note_filepath = os.path.join(self.save_dir, note_filename)
                        with open(note_filepath, 'w') as note_file:
                            note_file.write(note)

                        self.get_logger().info(f"Note générée et sauvegardée : {note_filepath}")
                        self.destination_set = False  # Réinitialiser la destination pour éviter de recréer la note

            # Afficher l'image avec OpenCV ('Camera Feed' s'ouvre en fentre principale)
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)

    # Créer une instance de notre noeud
    camera_subscriber = CameraSubscriber()

    # Exécuter le noeud ROS 2
    rclpy.spin(camera_subscriber)

    # Nettoyer
    camera_subscriber.destroy_node()
    cv2.destroyAllWindows()  # Assurez-vous que la fenêtre OpenCV se ferme correctement
    rclpy.shutdown()

if __name__ == '__main__':
    main()
