import numpy as np
import control as ct
import matplotlib.pyplot as plt

# =====================================================================
# 1. PARAMÈTRES EXACTS DE TON PROJET (main.cpp & slash_controller.py)
# =====================================================================
g0, tau, l = 0.43, 3.0, 0.3
v_d = 2.0               # Vitesse désirée [m/s]
dt = 0.05               # 20 Hz (self.dt dans slash_controller.py)

# Paramètres Arduino (ctl_mode == 5 dans main.cpp)
vel_k2 = 8.242
vel_ubar = 3.89

# Gain LQR direction déjà dans ton slash_controller.py
K_dir = np.array([0.316, 0.538])
max_steering = 0.52     # [rad]

# =====================================================================
# 2. SYNTHÈSE DU FILTRE DE KALMAN (LQE) POUR L'ÉTAT LATÉRAL x = [y, theta]
# =====================================================================
A = np.array([
    [0.0, v_d],
    [0.0, 0.0]
])
B = np.array([
    [0.0],
    [v_d / l]
])
C = np.eye(2)           # Mesures LiDAR : y_mes = [laser_y, laser_theta]

# Bruit de mesure LiDAR estimé (écart-type : 5 cm sur y, 0.08 rad (~4.5°) sur theta)
sigma_y = 0.05
sigma_theta = 0.08
Rv = np.diag([sigma_y**2, sigma_theta**2])

# Incertitude sur le modèle cinématique (glissement, variation de vitesse)
G = np.eye(2)
Qw = np.diag([0.005, 0.02])

# Calcul du gain optimal L (2x2)
L, P, poles_obs = ct.lqe(A, G, C, Qw, Rv)
print("Matrice de gain de Kalman L (à copier dans slash_controller.py) :")
print(repr(np.round(L, 4)))
print("Pôles de l'observateur :", np.round(poles_obs, 3))

# =====================================================================
# 3. SIMULATION COMPARATIVE : SANS KALMAN vs AVEC KALMAN
# =====================================================================
def run_sim(use_kalman=True):
    np.random.seed(42)  # Même bruit pour comparer équitablement
    t_eval = np.arange(0, 6.0, dt)
    N = len(t_eval)

    # État vrai : [v, y, theta] (départ décalé de 0.4 m et 0.15 rad)
    v_reel, y_reel, theta_reel = 1.5, 0.40, 0.15
    x_hat = np.array([0.40, 0.15])  # Estimation initiale [y_hat, theta_hat]

    hist_reel = np.zeros((N, 2))
    hist_mes  = np.zeros((N, 2))
    hist_hat  = np.zeros((N, 2))
    hist_cmd  = np.zeros(N)

    for k in range(N):
        # 1. Lecture LiDAR bruitée
        y_mes = np.array([
            y_reel + np.random.normal(0, sigma_y),
            theta_reel + np.random.normal(0, sigma_theta)
        ])

        # 2. Choix du vecteur utilisé par le contrôleur
        x_ctrl = x_hat if use_kalman else y_mes

        # 3. Loi de commande direction (identique à ctl_autopilot)
        delta_cmd = float(np.clip(-np.dot(K_dir, x_ctrl), -max_steering, max_steering))

        # Sauvegarde
        hist_reel[k] = [y_reel, theta_reel]
        hist_mes[k]  = y_mes
        hist_hat[k]  = x_hat
        hist_cmd[k]  = delta_cmd

        # 4. Mise à jour du Filtre de Kalman (Euler à pas dt = 0.05 s)
        dx_hat = A @ x_hat + B.flatten() * delta_cmd + L @ (y_mes - C @ x_hat)
        x_hat = x_hat + dt * dx_hat

        # 5. Simulation physique NON-LINÉAIRE (Arduino mode 5 + cinématique)
        V_mot = np.clip(vel_ubar + vel_k2 * (v_d - v_reel), -8.0, 8.0)
        v_reel     += dt * ((g0 * V_mot - v_reel) / tau)
        y_reel     += dt * (v_reel * np.sin(theta_reel))
        theta_reel += dt * ((v_reel / l) * np.tan(delta_cmd))

    return t_eval, hist_reel, hist_mes, hist_hat, hist_cmd

t, reel_sans, mes_sans, _, cmd_sans = run_sim(use_kalman=False)
t, reel_avec, mes_avec, hat_avec, cmd_avec = run_sim(use_kalman=True)

# =====================================================================
# 4. GRAPHIQUES POUR TON POWERPOINT (SECTION 1.4.4)
# =====================================================================
fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

# Position latérale y
axes[0].plot(t, mes_avec[:, 0], '.', color='silver', label='Mesure LiDAR bruitée')
axes[0].plot(t, reel_avec[:, 0], 'k--', linewidth=1.5, label='Position réelle y')
axes[0].plot(t, hat_avec[:, 0], 'b-', linewidth=2, label='Estimée Kalman y_hat')
axes[0].set_ylabel("Position y (m)")
axes[0].set_title("Validation du Filtre de Kalman (Auto-pilote Option #2)")
axes[0].legend(); axes[0].grid(True)

# Orientation theta
axes[1].plot(t, mes_avec[:, 1], '.', color='silver', label='Mesure LiDAR bruitée')
axes[1].plot(t, reel_avec[:, 1], 'k--', linewidth=1.5, label='Angle réel θ')
axes[1].plot(t, hat_avec[:, 1], 'g-', linewidth=2, label='Estimé Kalman θ_hat')
axes[1].set_ylabel("Angle θ (rad)")
axes[1].legend(); axes[1].grid(True)

# Commande de braquage delta (l'impact concret sur le servomoteur !)
axes[2].plot(t, cmd_sans, color='salmon', alpha=0.8, label='Commande δ SANS Kalman (vibrations servo)')
axes[2].plot(t, cmd_avec, 'b-', linewidth=2, label='Commande δ AVEC Kalman (lisse)')
axes[2].set_ylabel("Braquage δ (rad)")
axes[2].set_xlabel("Temps (s)")
axes[2].legend(); axes[2].grid(True)

plt.tight_layout()
plt.show()