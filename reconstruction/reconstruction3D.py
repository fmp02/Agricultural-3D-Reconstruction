#!/usr/bin/env python3

""" Codice per la ricostruzione 3d del campo agricolo a partire dai dati acquisiti dal robot e salvati nel dataset"""
import os
import json
import numpy as np
import open3d as o3d

DATASET  = "reconstruction_data"    # dataset in cui sono salvati i dati per ricostruire la scena
OUT_FILE = "fusion_final.ply"       # finestra output di open3d

# conta i frame presenti nel dataset (num rgb = num depth)
N_FRAMES = len([f for f in os.listdir(os.path.join(DATASET, "color"))])

# Caricamento dei parametri della camera salvati durante la fase di acquisizione
with open(os.path.join(DATASET, "camera_intrinsics.json"), "r") as f:
    K = json.load(f)

# Creazione del modello pinhole per la proiezione dei pixel nello spazio 3D
intrinsic = o3d.camera.PinholeCameraIntrinsic(
    K["width"], K["height"],
    K["fx"],    K["fy"],
    K["cx"],    K["cy"]
)

# Trasformazione dal sistema di riferimento della camera al sistema di riferimento del robot

# Mappatura
# X_robot (Avanti)   =  Z_camera (Profondità)
# Y_robot (Sinistra) = -X_camera (Destra invertita)
# Z_robot (Alto)     = -Y_camera (Basso invertito)

T_cam_to_robot = np.array([
    [ 0.0,  0.0,  1.0,  0.0],
    [-1.0,  0.0,  0.0,  0.0],
    [ 0.0, -1.0,  0.0,  0.0],
    [ 0.0,  0.0,  0.0,  1.0]
])

# Nuvola di punti globale contiene la fusione di tutti i frame
global_cloud = o3d.geometry.PointCloud()

#Per ogni dato acquisito:
    #1. carica RGB, depth e posa
    #2. genera la nuvola di punti locale
    #3. trasforma la nuvola nel sistema globale
    #4. la aggiunge alla ricostruzione finale
for i in range(N_FRAMES):

    color_path = os.path.join(DATASET, "color", f"frame_{i:05d}.png")
    depth_path = os.path.join(DATASET, "depth", f"frame_{i:05d}.npy")
    pose_path  = os.path.join(DATASET, "poses", f"frame_{i:05d}.txt")

    if not os.path.exists(color_path):
        continue

    print(f"\rFrame {i+1}/{N_FRAMES}", end="", flush=True)

    # Caricamento dell' RGB e della depth
    color = o3d.io.read_image(color_path)
    depth_np = np.load(depth_path)
    depth = o3d.geometry.Image(depth_np.astype(np.float32))

    # Creazione dell'immagine RGB-D per la ricostruzione 3D
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth,
        depth_scale=1.0,
        depth_trunc=10.0,
        convert_rgb_to_intensity=False
    )

    # Generazione della nuvola di punti nel sistema di riferimento della camera
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

    # Trasformazione da s.d.r della camera a s.d.r del robot
    pcd.transform(T_cam_to_robot)

    # Trasformazione da s.d.r del robot a s.d.r di world
    T_robot_world = np.loadtxt(pose_path)
    pcd.transform(T_robot_world)

    # Aggiunta nuvola di punti locale a quella globale
    global_cloud += pcd

# Elimina punti isolati dovuti a rumore nelle misure di profondità
global_cloud = global_cloud.voxel_down_sample(voxel_size=0.02)

global_cloud, _ = global_cloud.remove_statistical_outlier(
    nb_neighbors=10,
    std_ratio=2.0
)

# Salva la nuvola di punti finale
o3d.io.write_point_cloud(OUT_FILE, global_cloud)
print(f"\nSalvato → {OUT_FILE}")

# Visualizza il risultato della fusione
o3d.visualization.draw_geometries(
    [global_cloud],
    window_name="Fusion Final",
    width=1280,
    height=720
)