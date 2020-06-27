global Drone
clc;

% Parâmetros de estrutura
massa = 1.24706; % Massa (kg)
arm = 0.2; % Braço do drone (m)
ix = 12425673.83*10^(-9); % Momento de inércia do drone em relação ao eixo x (kg.m^2)
iy = 12828566.44*10^(-9); % Momento de inércia do drone em relação ao eixo y (kg.m^2)
iz = 24028440.95*10^(-9); % Momento de inércia do drone em relação ao eixo z (kg.m^2)

% Parâmetros de aeroprop (com base no artigo [2] da referência bibliográfica do relatório final)
kf = .0000313; % Coeficiente de força (N/(rad/s)^2)
km = .00000075; % Coeficiente de momento (N.m/(rad/s)^2)
Jr = .00006; % Momento de inércia do rotor (kg.m^2)

% Parâmetros do ambiente
grav = 9.81; % Gravidade (m/s^2)
Drone.G = [0; 0; grav];
Drone.t_amost = .01;

% Tensor de estados do DRONE EM RELAÇÃO A TERRA
Drone.X(:,:,1) = [0 0 0;                % x xdot xdotdot
                  0 0 0;                % y ydox ydotdot
                  0 0 0];               % z zdot zdotdot
Drone.X(:,:,2) = [0 0 0;                % phi phidot phidotdot
                  0 0 0;                % theta thetadox thetadotdot
                  0 0 0];               % psi psidot psidotdot

% Momentos de inércia
Drone.I = [ix 0 0;
           0 iy 0;
           0 0 iz];

%% Equações de Torque e Thrust
K = [kf kf kf kf;
    kf*arm*sqrt(2)/2 -kf*arm*sqrt(2)/2 -kf*arm*sqrt(2)/2 kf*arm*sqrt(2)/2;
    kf*arm*sqrt(2)/2 kf*arm*sqrt(2)/2 -kf*arm*sqrt(2)/2 -kf*arm*sqrt(2)/2;
    km -km km -km];
w_zero = sqrt(massa*grav/(4*kf));        % velocidade angular para sustentar o drone
Drone.w = [1; -1; 1; -1]*w_zero;        % velocidade angulares dos motores
Drone.U = K * Drone.w.^2;

%% Acelerações
% lineares DO DRONE EM RALAÇÃO A TERRA
Drone.Fb = [0; 0; (-1)*Drone.U(1)];
Drone.X(:,3,1) = Drone.G + rotation(Drone.X(1,1,2),Drone.X(2,1,2),Drone.X(3,1,2))*Drone.Fb/massa;

% angulares do DRONE EM REALAÇÃO A TERRA , sentido anti-horário POSITIVO
Drone.wR = sum(Drone.w);
Drone.X(:,3,2) = Drone.I^(-1)* (Drone.U(2:4) - cross(Drone.X(:,2,2), Drone.I*Drone.X(:,2,2) + [0; 0; Jr*Drone.wR]));

%% Atualização de Velocidade e Posição utilizando tempo de amostragem
% lineares
Drone.X(:,2,1) = Drone.X(:,2,1) + Drone.X(:,3,1)*Drone.t_amost;
Drone.X(:,1,1) = Drone.X(:,1,1) + Drone.X(:,2,1)*Drone.t_amost;

% angulares
Drone.X(:,2,2) = Drone.X(:,2,2) + Drone.X(:,3,2)*Drone.t_amost;
Drone.X(:,1,2) = Drone.X(:,1,2) + Drone.X(:,2,2)*Drone.t_amost;
R1 = rotation(0,0,0)';
R2 = rotation(Drone.X(1,1,2),0,0)';
R3 = rotation(Drone.X(1,1,2),Drone.X(2,1,2),0)';
Drone.pqr = [R1(:,1), R2(:,2), R3(:,3)]*Drone.X(:,2,2);
