function [Ad, Bd, Cd, Dd] = LPV_cont_discrete(states)
    % This is an LPV model concerning the three position and three rotational axis.
    
    % WARNING! Matrix sizes hard-coded!
    
    % Get the constants from the general pool of constants
    constants = initial_constants();
    m=constants('m');
    Iz=constants('Iz');
    Caf=constants('Caf');
    Car=constants('Car');
    lf=constants('lf');
    lr=constants('lr');
    Ts=constants('Ts');
    x_dot=constants('x_dot');
    
    psi = states(2);
    Y=states(4);
    

    %% Get the LPV matrices for the control

    A=[-(2*Caf+2*Car)/(m*x_dot) 0 (-x_dot-(2*Caf*lf-2*Car*lr)/(m*x_dot)) 0; ...
        0 0 1 0; ...
        -(2*lf*Caf-2*lr*Car)/(Iz*x_dot) 0 -(2*lf^2*Caf+2*lr^2*Car)/(Iz*x_dot) 0; ...
        1 x_dot 0 0];

    B=[2*Caf/m;0;2*lf*Caf/Iz;0];
    
    C= [0 1 0 0;0 0 0 1];
    D=0;


    %% Discretize the system
    
    % Forward Euler
    Ad=eye(length(A(1,:)))+Ts*A;
    Bd=Ts*B;
    Cd=C;
    Dd=D;
    
end

            
            