import torch
import torch.nn as nn

class SupportFilesCar:
    ''' The following functions interact with the main file'''

    def __init__(self, device):
        ''' Load the constants that do not change'''

        # Constants
        g = 9.81
        m = 1500
        Iz = 3000
        Cf = 38000
        Cr = 66000
        lf = 2
        lr = 3
        Ts = 0.02
        mju = 0.02 # friction coefficient

        ####################### Lateral control #################################

        outputs = 4 # number of outputs
        inputs = 2 # number of inputs
        hz = 10 # horizon period

        trajectory = 2 # Choose 1, 2 or 3, nothing else
        version = 2 # This is only for trajectory 3 (Choose 1 or 2)

        # Matrix weights for the cost function (They must be diagonal)

        if trajectory == 3 and version == 2:  # 3rd trajectoris with larger distances
            # Weights for trajectory 3, version 2
            Q = torch.tensor([[100, 0, 0, 0],[0, 20000, 0, 0],[0, 0, 1000, 0],[0, 0, 0, 1000]]).to(device) # weights for outputs (all samples, except the last one)
            S = torch.tensor([[100, 0, 0, 0],[0, 20000, 0, 0],[0, 0, 1000, 0],[0, 0, 0, 1000]]).to(device) # weights for the final horizon period outputs
            R = torch.tensor([[100, 0],[0, 1]]).to(device) # weights for inputs
        elif trajectory == 3:
            # Weights for trajectory 3, version 1
            Q = torch.tensor([[100, 0, 0, 0],[0, 20000, 0, 0],[0, 0, 1000, 0],[0, 0, 0, 1000]]).to(device) # weights for outputs (all samples, except the last one)
            S = torch.tensor([[100, 0, 0, 0],[0, 20000, 0, 0],[0, 0, 1000, 0],[0, 0, 0, 1000]]).to(device) # weights for the final horizon period outputs
            R = torch.tensor([[100, 0],[0, 1]]).to(device) # weights for inputs
        else:
            # Weights for trajectories 1 & 2
            Q = torch.tensor([[1, 0, 0, 0],[0, 200, 0, 0],[0, 0, 50, 0],[0, 0, 0, 50]]).to(device) # weights for outputs (all samples, except the last one)
            S = torch.tensor([[1, 0, 0, 0],[0, 200, 0, 0],[0, 0, 50, 0],[0, 0, 0, 50]]).to(device) # weights for the final horizon period outputs
            R = torch.tensor([[100, 0],[0, 1]]).to(device) # weights for inputs

        # Please do not modify the time_length!
        delay = 0
        if trajectory == 1:
            time_length = 60. # simultion length
            x_lim = 1000   # x and y limits
            y_lim = 1000
        elif trajectory == 2:
            time_length = 140.
            x_lim = 1000
            y_lim = 1000
        elif trajectory == 3:
            if version == 1:
                x_lim = 170
                y_lim = 160
            else:
                x_lim = 170 * version
                y_lim = 160 * version

            # since trajectry 3 is made up of 11 different trajectories we define time length for 1st and other trajectoris
            first_section = 14  # 14 sec for first part of the trajectory
            other_sections = 14  # 14 sec for other parts of the trajectories
            time_length = first_section + other_sections * 10  # total time length of 11 trajectories
            delay = torch.zeros(12)

            # this loop defines the time start and end of each trajectories
            for i in range(1, 12):
                delay[i] = delay[i-1] + 14

        Qf = S # Final cost
        # This is the control horizon
        n = 10

        # Vehicle model
        # state space model
        A = torch.tensor([[0, 1, 0, 0],[0, -(2*Cf+2*Cr)/(m*3), (2*Cf+2*Cr)/m, 0],[0, 0, 0, 1],[0, (2*lf*Cf-2*lr*Cr)/(Iz*3), (2*lr*Cr-2*lf*Cf)/Iz, 0]]).to(device)
        B = torch.tensor([[0],[2*Cf/(m*3)],[0],[2*lf*Cf/Iz]]).to(device)

        # Pole placement
        r0 = 0.8
        r1 = 0.8
        r2 = 0.8
        r3 = 0.8
        # pole placement
        K = nn.Parameter(torch.tensor([[r0, 0, 0, 0],[0, r1, 0, 0],[0, 0, r2, 0],[0, 0, 0, r3]])).to(device)
        ############################ Lateral control #########################################

        self.data = (g, m, Iz, Cf, Cr, lf, lr, Ts, mju, outputs, inputs, hz, trajectory, version, Q, S, R, Qf, n, A, B, delay, x_lim, y_lim, first_section, other_sections, time_length, K)

    def update_parameters(self,):
        pass

    def lateral_control(self, trajectory):
        ''' Loads all required functions and performs the lateral control'''

        ################### Initializations ################################################
        g, m, Iz, Cf, Cr, lf, lr, Ts, mju, outputs, inputs, hz, trajectory, version, Q, S, R, Qf, n, A, B, delay, x_lim, y_lim, first_section, other_sections, time_length, K = self.data

        # Time variables
        t = 0
        dt = 0.02
        t0 = delay[trajectory]
        t_index = int(t0/dt)
        t_end = (time_length+delay[trajectory])/dt
        t_vector = torch.arange(t0, t_end, dt).to(device)

        # State vector
        x = torch.zeros((4, int(t_end))).to(device)

        # We start all lateral errors with zero
        y_e = torch.zeros((outputs, n, int(t_end))).to(device)
        y_f_e = torch.zeros((outputs, int(t_end))).to(device)
        u_e = torch.zeros((inputs, n, int(t_end))).to(device)
        u_f_e = torch.zeros((inputs, int(t_end))).to(device)

        # Vector for saving the data
        simdata = torch.zeros((6, int(t_end))).to(device)

        ###################### Closed loop system ############################################
        while t < t_end:

            # State vector
            x[:, [t_index]] = x[:, [t_index-1]] + dt * (A @ x[:, [t_index-1]] + B @ u_e[:, [0], [t_index-1]])

            # Lateral error
            y_e[:, :, [t_index]] = y_e[:, :, [t_index-1]]

            # Feed forward lateral error
            y_f_e[:, [t_index]] = torch.tensor([0, 0, trajectory * t_vector[t_index]**2 / other_sections, 0]).to(device)

            # Feed forward lateral error
            y_f_e[:, [t_index]] = y_f_e[:, [t_index]] - torch.tensor([0, 0, trajectory * (t_vector[t_index-dt])**2 / other_sections, 0]).to(device)

            # States and feed forward signal
            if t_index >= n:
                y_e[:, :, [t_index]] = y_e[:, :, [t_index]] - y_f_e[:, [t_index-n]]
            else:
                y_e[:, :, [t_index]] = y_e[:, :, [t_index]]

            # Lateral force
            u_e[:, :, [t_index]] = u_e[:, :, [t_index-1]]

            # Define u for calculating the lateral force
            u_e[:, [0], [t_index]] = R.inverse() @ (B.T @ (K @ y_e[:, :, [t_index]] + y_f_e[:, [t_index]] - Q @ x[:, [t_index]]))

            # Define the lateral force
            u_f_e[:, [t_index]] = u_e[:, [0], [t_index]] * trajectory
            u_f_e[:, [t_index]] = u_f_e[:, [t_index]] - u_f_e[:, [t_index-dt]]

            # Save the states and inputs for the output
            simdata[:, [t_index]] = torch.cat((x[:, [t_index]], u_f_e[:, [t_index]]), 0)

            # Increase time
            t += dt
            t_index += 1

        return simdata

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
support_files_car = SupportFilesCar(device)

# trajectory = 3  # Choose 1, 2, or 3
# simdata = support_files_car.lateral_control(trajectory)
# print(simdata)
