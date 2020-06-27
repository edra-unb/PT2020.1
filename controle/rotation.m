
%% Matriz de rotação
function Y = rotation(phi,theta,psi)
Y = [cos(psi)*cos(theta) cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi) cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi);...
    sin(psi)*cos(theta) sin(phi)*sin(psi)*sin(theta)+cos(phi)*cos(psi) cos(phi)*sin(psi)*sin(theta)-sin(phi)*cos(psi);...
    -sin(theta) cos(theta)*sin(phi) cos(phi)*cos(theta)];

end