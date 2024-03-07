close all
clear all
clc
warning off;

%% Load the constant values
constants=initial_constants();
Ts=constants('Ts');
controlled_states=constants('controlled_states'); % number of controlled states in this script
hz = constants('hz');
x_dot=constants('x_dot');
trajectory=constants('trajectory');

%% Generate the reference signals
t = 0:Ts:7;
[psi_ref,X_ref,Y_ref]=trajectory_generator(t);

sim_length=length(t); % Number of control loop iterations

refSignals=zeros(length(X_ref(:,2))*controlled_states,1);
k=1;
for i = 1:controlled_states:length(refSignals)
   refSignals(i)=psi_ref(k,2);
   refSignals(i+1)=Y_ref(k,2);
   k=k+1;
end
clear i k

%% Load the initial state

y_dot=0;
psi=psi_ref(1,2);
psi_dot=0;
if trajectory==2
    Y=3;
else
    Y=Y_ref(1,2);
end

states=[y_dot,psi,psi_dot,Y];
statesTotal=zeros(length(t),length(states));
statesTotal(1,:)=states;
%% Initiate the controller - simulation loops

U1=0; % Input at t = -1 s (delta)

UTotal=zeros(length(t),1); % 1 inputs
UTotal(1,:)=U1;

% Generate the discrete state space matrices
[Ad, Bd, Cd, Dd]=LPV_cont_discrete(states);

% UPDATE FROM THE VIDEO EXPLANATIONS:
% Generate the compact simplification matrices for the cost function
% The matrices (Hdb,Fdbt,Cdb,Adc) stay mostly constant during the simulation.
% Therefore, it is more efficient to generate them here before you start the simulation loop.
% However, in the end of the simulation, the horizon period (hz) will start decreasing.
% That is when the matrices need to be regenerated (done inside the simulation loop)
[Hdb,Fdbt,Cdb,Adc]=MPC_simplification(Ad,Bd,Cd,Dd,hz);

k=1; % for reading reference signals
for i =1:sim_length-1
    
    
    %% Generating the current state and the reference vector
    x_aug_t=[states';U1];

    k=k+controlled_states;
    
    if k+controlled_states*hz-1 <= length(refSignals)
        r=refSignals(k:k+controlled_states*hz-1);
    else
        r=refSignals(k:length(refSignals));
        hz=hz-1;
    end
    
    if hz<constants('hz')
        % Check if hz starts decreasing
        % These matrices (Hdb,Fdbt,Cdb,Adc) were created earlier at the beginning of the loop.
        % They constant almost throughout the entire simulation. However,
        % in the end of the simulation, the horizon period (hz) starts decreasing.
        % Therefore, the matrices need to be constantly updated in the end of the simulation.
        [Hdb,Fdbt,Cdb,Adc]=MPC_simplification(Ad,Bd,Cd,Dd,hz);
    end
    
    %% Calling the optimizer (quadprog)
    
    % Cost function in quadprog: min(du)*1/2*du'Hdb*du+f'du
    ft=[x_aug_t',r']*Fdbt;

    % Hdb must be positive definite for the problem to have finite minimum.

    % Call the solver
    options = optimset('Display', 'off');
    [du,fval]=quadprog(Hdb,ft,[],[],[],[],[],[],[],options);

    % Update the real inputs
    U1=U1+du(1);
    
    if U1 < -pi/6
        U1=-pi/6;
    elseif U1>pi/6
        U1=pi/6;
    else
        U1=U1;
    end
    
    UTotal(i+1,:)=U1;
    
    % Simulate the new states
    T = (Ts)*(i-1):(Ts)/30:Ts*(i-1)+(Ts);
    [T,x]=ode45(@(t,x) nonlinear_lateral_car_model(t,x,U1),T,states);
    states=x(end,:);
    statesTotal(i+1,:)=states;
    
end

%% Plot the trajectory

% Trajectory
figure;
plot(X_ref(:,2),Y_ref(:,2),'--b','LineWidth',2)
hold on
plot(X_ref(:,2),statesTotal(1:end,4),'r','LineWidth',1)
grid on;
xlabel('x-position [m]','FontSize',15)
ylabel('y-position [m]','FontSize',15)
legend({'position-ref','position'},'Location','northeast','FontSize',15)

%% Plot the position states and the yaw

% Plot references individually
figure;
subplot(2,1,1)
plot(t(1:sim_length),Y_ref(1:sim_length,2),'--b','LineWidth',2)
hold on
subplot(2,1,2)
plot(t(1:sim_length),psi_ref(1:sim_length,2),'--b','LineWidth',2)

hold on
subplot(2,1,1)
plot(t(1:sim_length),statesTotal(1:sim_length,4),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('y-position [m]','FontSize',15)
legend({'y-ref','y-position'},'Location','northeast','FontSize',15)
subplot(2,1,2)
plot(t(1:sim_length),statesTotal(1:sim_length,2),'r','LineWidth',1)
grid on
legend({'psi-ref','psi-position'},'Location','northeast','FontSize',15)
xlabel('time [s]','FontSize',15)
ylabel('psi-position [rad]','FontSize',15)

%% Plot the inputs
figure;
plot(t(1:length(statesTotal(:,1))),UTotal(:,1))
grid on
xlabel('time [s]','FontSize',15)
ylabel('U2 [rad]','FontSize',15)
