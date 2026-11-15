import numpy as np
import control as ct
import matplotlib.pyplot as plt

# =====================================================================
# 1. PARAMÈTRES ET MODÈLE COMPLET À 3 ÉTATS : x = [v, y, theta]^T
# =====================================================================
g0, tau, l = 0.43, 3.0, 0.3
v_d = 2.0               # Vitesse désirée [m/s]
V_0 = v_d / g0          # Voltage d'équilibre (~4.65 V)
dt = 0.05               # Pas de temps [s]

A = np.array([
    [-1/tau, 0.0, 0.0],
    [0.0,    0.0, v_d],
    [0.0,    0.0, 0.0]
])
B = np.array([
    [g0/tau, 0.0  ],
    [0.0,    0.0  ],
    [0.0,    v_d/l]
])
# C (2x3) : On mesure UNIQUEMENT v (encodeur) et y (LiDAR). Theta n'est pas mesuré !
C = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0]
])

# =====================================================================
# 2. SYNTHÈSE DU CONTRÔLEUR LQR (Section 1.4.1)
# =====================================================================
Q_lqr = np.diag([100.0, 1.0, 1.0])
R_lqr = np.diag([1.0, 10.0])
K, _, _ = ct.lqr(A, B, Q_lqr, R_lqr)

# =====================================================================
# 3. SYNTHÈSE DU FILTRE DE KALMAN LQE (Section 1.4.4)
# =====================================================================
# Bruit de mesure capteurs : sigma_v = 0.12 m/s (encodeur), sigma_y = 0.05 m (LiDAR)
sigma_v, sigma_y = 0.12, 0.05
Rv = np.diag([sigma_v**2, sigma_y**2])

# Incertitude sur le modèle [v, y, theta]
G = np.eye(3)
Qw = np.diag([0.04, 0.005, 0.02])

# Calcul du gain optimal L (taille 3x2)
L, P, poles_obs = ct.lqe(A, G, C, Qw, Rv)
print("Matrice de gain de Kalman L (3x2) :")
print(repr(np.round(L, 4)))
print("Pôles de l'observateur :", np.round(poles_obs, 3))

# =====================================================================
# 4. SIMULATION NON-LINÉAIRE : CONTRÔLEUR LQR + FILTRE DE KALMAN
# =====================================================================
np.random.seed(42)
t_eval = np.arange(0, 6.0, dt)
N = len(t_eval)

# État réel initial : [v, y, theta]
x_reel = np.array([1.4, 0.45, 0.20])
# État estimé initial : theta est inconnu au départ -> initialisé à 0.0
x_hat  = np.array([1.4, 0.45, 0.00])

hist_reel = np.zeros((N, 3))
hist_mes  = np.zeros((N, 2))
hist_hat  = np.zeros((N, 3))
hist_u    = np.zeros((N, 2))

for k in range(N):
    # 1. Mesures capteurs bruitées : y_mes = [v_enc, y_lidar]^T
    y_mes = C @ x_reel + np.array([
        np.random.normal(0, sigma_v),
        np.random.normal(0, sigma_y)
    ])

    hist_reel[k] = x_reel
    hist_mes[k]  = y_mes
    hist_hat[k]  = x_hat

    # 2. Commande LQR basée sur l'état estimé x_hat = [v_hat, y_hat, theta_hat]^T
    x_tilde_hat = np.array([x_hat[0] - v_d, x_hat[1], x_hat[2]])
    u_tilde = -K @ x_tilde_hat

    V     = float(np.clip(V_0 + u_tilde[0], -8.0, 8.0))
    delta = float(np.clip(u_tilde[1], -0.52, 0.52))
    hist_u[k] = [V, delta]

    # 3. Mise à jour du Filtre de Kalman (Euler)
    y_tilde_mes = np.array([y_mes[0] - v_d, y_mes[1]])
    x_tilde_curr = np.array([x_hat[0] - v_d, x_hat[1], x_hat[2]])
    u_tilde_sat = np.array([V - V_0, delta])

    dx_hat = A @ x_tilde_curr + B @ u_tilde_sat + L @ (y_tilde_mes - C @ x_tilde_curr)
    x_hat = x_hat + dt * dx_hat

    # 4. Évolution physique NON-LINÉAIRE du véhicule
    v, y, theta = x_reel
    x_reel = x_reel + dt * np.array([
        (g0 * V - v) / tau,
        v * np.sin(theta),
        (v / l) * np.tan(delta)
    ])

# =====================================================================
# 5. GRAPHIQUES DE VALIDATION (2x2)
# =====================================================================
fig, axes = plt.subplots(2, 2, figsize=(12, 8))

# 1. Vitesse v (mesurée par encodeur)
axes[0, 0].plot(t_eval, hist_mes[:, 0], '.', color='silver', label='Mesure bruitée (Encodeur)')
axes[0, 0].plot(t_eval, hist_reel[:, 0], 'k--', linewidth=1.5, label='Vitesse réelle v')
axes[0, 0].plot(t_eval, hist_hat[:, 0], 'b-', linewidth=2, label='Estimée Kalman v_hat')
axes[0, 0].set_title("1. Vitesse v (m/s) — Filtrage encodeur")
axes[0, 0].legend()
axes[0, 0].grid(True)

# 2. Position latérale y (mesurée par LiDAR)
axes[0, 1].plot(t_eval, hist_mes[:, 1], '.', color='silver', label='Mesure bruitée (LiDAR)')
axes[0, 1].plot(t_eval, hist_reel[:, 1], 'k--', linewidth=1.5, label='Position réelle y')
axes[0, 1].plot(t_eval, hist_hat[:, 1], 'g-', linewidth=2, label='Estimée Kalman y_hat')
axes[0, 1].set_title("2. Position latérale y (m) — Filtrage LiDAR")
axes[0, 1].legend()
axes[0, 1].grid(True)

# 3. Angle theta (NON MESURÉ — Reconstruit par l'observateur !)
axes[1, 0].plot(t_eval, hist_reel[:, 2], 'k--', linewidth=1.5, label='Angle réel θ')
axes[1, 0].plot(t_eval, hist_hat[:, 2], 'r-', linewidth=2, label='Reconstruit Kalman θ_hat (sans capteur)')
axes[1, 0].set_title("3. Orientation θ (rad) — État caché reconstruit")
axes[1, 0].set_xlabel("Temps (s)")
axes[1, 0].legend()
axes[1, 0].grid(True)

# 4. Commandes u = [V, delta]
axes[1, 1].plot(t_eval, hist_u[:, 0], 'b-', label='Voltage moteur V (V)')
axes[1, 1].plot(t_eval, hist_u[:, 1] * 10, 'g-', label='Braquage δ × 10 (rad)')
axes[1, 1].set_title("4. Commandes LQR issues de l'état filtré")
axes[1, 1].set_xlabel("Temps (s)")
axes[1, 1].legend()
axes[1, 1].grid(True)

plt.tight_layout()
plt.show()
