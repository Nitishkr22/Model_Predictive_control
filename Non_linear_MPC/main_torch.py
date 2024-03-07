import numpy as np
import torch
import matplotlib.pyplot as plt
import support_files_car_general as sfc_g
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from qpsolvers import solve_qp

# Set up GPU if available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Create an object for the support functions
support = sfc_g.SupportFilesCar()
constants = support.constants

# Load the constant values needed in the main file
Ts = constants['Ts']
outputs = constants['outputs']  # number of outputs (psi, Y)
hz = constants['hz']  # horizon prediction period
time_length = constants['time_length']  # duration of the manoeuvre
inputs = constants['inputs']
x_lim = constants['x_lim']
y_lim = constants['y_lim']
trajectory = constants['trajectory']

# Generate the reference signals
t = np.zeros((int(time_length / Ts + 1)))
for i in range(1, len(t)):
    t[i] = np.round(t[i - 1] + Ts, 2)

x_dot_ref, y_dot_ref, psi_ref, X_ref, Y_ref = support.trajectory_generator(t)
sim_length = len(t)  # Number of control loop iterations
refSignals = np.zeros(len(X_ref) * outputs)

# Build up the reference signal vector
k = 0
for i in range(0, len(refSignals), outputs):
    refSignals[i] = x_dot_ref[k]
    refSignals[i + 1] = psi_ref[k]
    refSignals[i + 2] = X_ref[k]
    refSignals[i + 3] = Y_ref[k]
    k += 1

# Load the initial states
x_dot = x_dot_ref[0]
y_dot = y_dot_ref[0]
psi = psi_ref[0]
psi_dot = 0.
X = X_ref[0]
Y = Y_ref[0]

states = np.array([x_dot, y_dot, psi, psi_dot, X, Y])
statesTotal = np.zeros((len(t), len(states)))
statesTotal[0][0:len(states)] = states

# Load the initial input
U1 = 0  # Input at t = -0.02 s (steering wheel angle in rad (delta))
U2 = 0  # Input at t = -0.02 s (acceleration in m/s^2 (a))
UTotal = np.zeros((len(t), 2))  # To keep track all your inputs over time
UTotal[0][0] = U1
UTotal[0][1] = U2

# Initiate the controller - simulation loops
du = np.zeros((inputs * hz, 1))

# Arrays for the animation
t_ani = []
x_dot_ani = []
psi_ani = []
X_ani = []
Y_ani = []
delta_ani = []

for i in range(0, sim_length - 1):
    # Generate the discrete state space matrices
    Ad, Bd, Cd, Dd = support.state_space(states, U1, U2)

    # Convert to PyTorch tensors and move to GPU
    Ad_tensor = torch.tensor(Ad, device=device, dtype=torch.float32)
    Bd_tensor = torch.tensor(Bd, device=device, dtype=torch.float32)
    Cd_tensor = torch.tensor(Cd, device=device, dtype=torch.float32)
    Dd_tensor = torch.tensor(Dd, device=device, dtype=torch.float32)
    x_aug_t_tensor = torch.tensor(
        np.concatenate((states, [U1, U2]), axis=0), device=device, dtype=torch.float32
    )

    # Generate the compact simplification matrices for the cost function
    Hdb, Fdbt, Cdb, Adc, G, ht = support.mpc_simplification(
        Ad_tensor, Bd_tensor, Cd_tensor, Dd_tensor, hz, x_aug_t_tensor, du
    )

    # Convert PyTorch tensors to NumPy arrays
    Hdb_np = Hdb.cpu().numpy()
    Fdbt_np = Fdbt.cpu().numpy()
    Cdb_np = Cdb.cpu().numpy()
    Adc_np = Adc.cpu().numpy()
    G_np = G.cpu().numpy()
    ht_np = ht.cpu().numpy()

    # Solve the quadratic program
    ft = np.matmul(
        np.concatenate((np.transpose(x_aug_t_tensor)[0][0 : len(x_aug_t_tensor)], r), axis=0),
        Fdbt_np,
    )
    try:
        du = solve_qp(Hdb_np, ft, G_np, ht_np, solver="cvxopt")
        du = np.transpose([du])
    except ValueError as ve:
        print("Error:", ve)
        break

    # Update the real inputs
    U1 += du[0][0]
    U2 += du[1][0]

    UTotal[i + 1][0] = U1
    UTotal[i + 1][1] = U2

    states, x_dot_dot, y_dot_dot, psi_dot_dot = support.open_loop_new_states(
        states, U1, U2
    )
    statesTotal[i + 1][0 : len(states)] = states

    # Append data for animation
    if i % 5 == 1:
        t_ani = np.concatenate([t_ani, [t[i]]])
        x_dot_ani = np.concatenate([x_dot_ani, [states[0]]])
        psi_ani = np.concatenate([psi_ani, [states[2]]])
        X_ani = np.concatenate([X_ani, [states[4]]])
        Y_ani = np.concatenate([Y_ani, [states[5]]])
        delta_ani = np.concatenate([delta_ani, [U1]])

