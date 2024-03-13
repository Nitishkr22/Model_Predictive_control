import numpy as np
import matplotlib.pyplot as plt
import mpc_support_new2 as sfc_g
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from qpsolvers import *
np.set_printoptions(suppress=True)

# Create an object for the support functions.
support=sfc_g.SupportFilesCar()
constants=support.constants

# Load the constant values needed in the main file
Ts=constants['Ts']
outputs=constants['outputs'] # number of outputs (psi, Y)
hz = constants['hz'] # horizon prediction period
time_length=constants['time_length'] # duration of the manoeuvre
inputs=constants['inputs']
x_lim=constants['x_lim']
y_lim=constants['y_lim']
trajectory=constants['trajectory']

# Generate the refence signals
t=np.zeros((int(time_length/Ts+1)))
for i in range(1,len(t)):
    t[i]=np.round(t[i-1]+Ts,2)

# print(len(t))
# exit()

x_dot_ref,y_dot_ref,psi_ref,X_ref,Y_ref=support.trajectory_generator(t)
sim_length=len(t) # Number of control loop iterations
print("sim_length: ",sim_length)   ##### sim_length = len(X_ref)
print(len(X_ref))
# exit()
refSignals=np.zeros(len(X_ref)*outputs)

# our reference array will be same like output array i.e. [x_dot,psi,X,Y] we have 4 outputs
# Build up the reference signal vector:
# refSignal = [x_dot_ref_0, psi_ref_0, X_ref_0, Y_ref_0, x_dot_ref_1, psi_ref_1, X_ref_1, Y_ref_1, x_dot_ref_2, psi_ref_2, X_ref_2, Y_ref_2, ... etc.]
k=0
for i in range(0,len(refSignals),outputs):
    refSignals[i]=x_dot_ref[k]
    refSignals[i+1]=psi_ref[k]
    refSignals[i+2]=X_ref[k]
    refSignals[i+3]=Y_ref[k]
    k=k+1

# Load the initial states
# If you want to put numbers here, please make sure that they are float and not
# integers. It means that you should add a point there.
# Example: Please write 0. in stead of 0 (Please add the point to make it float)

#load initial state same as the reference states


x_dot=x_dot_ref[0]
y_dot=y_dot_ref[0]
psi=psi_ref[0]
psi_dot=0.
X=X_ref[0]
Y=Y_ref[0]

states=np.array([x_dot,y_dot,psi,psi_dot,X,Y])
statesTotal=np.zeros((len(t),len(states))) #rows,columns # It will keep track of all your states during the entire manoeuvre
statesTotal[0][0:len(states)]=states  # initialise 1st state

######################### Accelerations ########################################

# initialise net accelerations
x_dot_dot=0.
y_dot_dot=0.
psi_dot_dot=0.

accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
accelerations_total=np.zeros((len(t),len(accelerations)))

################################################################################
# X_opt_total=np.zeros((len(t),hz))
# Y_opt_total=np.zeros((len(t),hz))

# Load the initial input

U1=0.0 # Input at t = -0.02 s (steering wheel angle in rad (delta))
U2=0.0 # Input at t = -0.02 s (acceleration in m/s^2 (a))

### next 3 lines not required in real time
UTotal=np.zeros((len(t),2)) # To keep track all your inputs over time for simulation purpose
UTotal[0][0]=U1
UTotal[0][1]=U2

# Initiate the controller - simulation loops
k=0
du=np.zeros((inputs*hz,1))  # this is du6 global, inputs=2

# # To extract X_opt from predicted x_aug_opt
# C_X_opt=np.zeros((hz,(len(states)+np.size(U1)+np.size(U2))*hz))
# for i in range(4,hz+4):
#     C_X_opt[i-4][i+7*(i-4)]=1
#
# # To extract Y_opt from predicted x_aug_opt
# C_Y_opt=np.zeros((hz,(len(states)+np.size(U1)+np.size(U2))*hz))
# for i in range(5,hz+5):
#     C_Y_opt[i-5][i+7*(i-5)]=1

# Arrays for the animation - every 5th state goes in there (once in 0.1 seconds, because Ts=0.02 seconds)

t_ani=[]
x_dot_ani=[]
psi_ani=[]
X_ani=[]
Y_ani=[]
delta_ani=[]
steer = []
dub = []
print(sim_length)
# exit()
X_last = refSignals[-2]
Y_last = refSignals[-1]
xy = [X_last,Y_last]
# print(Y_ref[0])
# exit()
wp = 0
while (True):
    try:
        if (wp == len(X_ref)-2):
            break
        print("waypoint index =============== ",wp)
        position = [states[-2],states[-1]]
        current_ref = [X_ref[wp],Y_ref[wp]]
        # position = [float(y_pos), float(x_pos)]
        print("aaaaaaaaaaaa: ",position)
        # print("Current position = ", position)
        if ((np.linalg.norm(np.array(position) - xy))>1):
            print("ddddddddddddddddddddddd")
            # Generate the discrete state space matrices from current state and current inputs
            
            Ad,Bd,Cd,Dd=support.state_space(states,U1,U2)

            # Generate the augmented current state and the reference vector
            x_aug_t=np.transpose([np.concatenate((states,[U1,U2]),axis=0)])

            k=k+outputs
            if k+outputs*hz<=len(refSignals):
                r=refSignals[k:k+outputs*hz]
            else:
                print("hello")
                r=refSignals[k:len(refSignals)]
                hz=hz-1
                print(k)

            Hdb,Fdbt,Cdb,Adc,G,ht=support.mpc_simplification(Ad,Bd,Cd,Dd,hz,x_aug_t,du)

            ft=np.matmul(np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)],r),axis=0),Fdbt)
            
            try:
                du=solve_qp(Hdb,ft,G,ht,solver="cvxopt")
                du=np.transpose([du])
                # print(du)
                # exit()
            except ValueError as ve:
                # print("dddddddddddddddddddddddd", ve)
                print(Hdb)
                print(ft)
                print(G)
                print(ht)
                print(Adc)
                print(x_aug_t)
                print(du)
                print(i)
                break

            steer_output =  U1*(180/np.pi)

            steer_output = np.clip(steer_output, a_min = -30, a_max = 30)
            steer_output = (50/3)*steer_output
            steer_output = np.clip(steer_output, a_min = -500, a_max = 500)
            print("steeering: ",steer_output)
            print("acceleration: ",U2)
            print("pppp: ",(np.linalg.norm(np.array(position) - current_ref)))
            # print("Hai+++++++++++++++++++++++++++")
            # steer.append(steer_output)
            

            # print(dub[-1][0][0])

            # 
            # print("duuu: ",du[0][0],du[1][0])
            # print(type(du))
            # print("Value of du:", du)
            # if du is None or du[0] is None:
            #     print("helloaa")
            #     # du[0][0] = 0
            #     # du[1][0] = 0
            #     # U1=U1+steer[-1]
            #     continue
            # else:
            
            U1=U1+du[0][0]
            U2=U2+du[1][0]
            
            # dub.append(du)
            UTotal[wp+1][0]=U1
            UTotal[wp+1][1]=U2
            print("Hai+++++++++++++++++++++++++++")
            states,x_dot_dot,y_dot_dot,psi_dot_dot=support.open_loop_new_states(states,U1,U2)
            statesTotal[wp+1][0:len(states)]=states
            print("updated: ",[states[-2],states[-1]])

            ######################### Accelerations ###################################
            accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
            accelerations_total[wp+1][0:len(accelerations)]=accelerations

            if i%500==0:
                print("Progress: "+str(round(wp/sim_length*100,2))+"%")

            # To make the animations 5x faster
            if i%5==1:
                t_ani=np.concatenate([t_ani,[t[wp]]])
                x_dot_ani=np.concatenate([x_dot_ani,[states[0]]])
                psi_ani=np.concatenate([psi_ani,[states[2]]])
                X_ani=np.concatenate([X_ani,[states[4]]])
                Y_ani=np.concatenate([Y_ani,[states[5]]])
                delta_ani=np.concatenate([delta_ani,[U1]])

            if (((np.linalg.norm(np.array(position) - current_ref)) <6.0 )):
                print("qqqqqqqqqqqqqqqqqqqqq")
                wp = wp + 1
            
            

            
    except:
        pass
