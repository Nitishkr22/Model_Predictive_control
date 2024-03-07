function constants=initial_constants()
    
    % Constants    
    g=9.81;
    m=1500;
    
    rho=1.225;
    Af=1.6+0.00056*(m-765);
    Cd=0.3;
    
    cd=0.5*rho*Cd*Af;
    cr=0.03;
    Iz=3000;
    Caf=19000;
    Car=33000;
    lf=1.2;
    lr=1.6;
    Ts=0.1;
    
    % Parameters for the lane change:
    % Higher psi reduces the overshoot
    % Matrix weights for the cost function (They must be diagonal)
    Q=[10 0;0 1]; % weights for outputs (output x output)
    S=[10 0;0 1]; % weights for the final horizon outputs (output x output)
    R=30; % weights for inputs (input x input)
    
    controlled_states=2;
    hz = 15; % horizon period
    x_dot=20;
    
    % Choose your trajectory (1,2)
    trajectory=1;
    
    keySet={'g','m','cd','cr','Iz','Caf','Car','lf','lr','Ts','Q','S','R','controlled_states','hz','x_dot','trajectory'};
    constants_list={g m cd cr Iz Caf Car lf lr Ts Q S R controlled_states hz x_dot trajectory};
    constants=containers.Map(keySet,constants_list);

end