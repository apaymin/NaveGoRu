function DCMnb_m = euler2dcm_nb_m(euler)
% euler2dcm: converts from Euler angles to DCM nav-to-body.
% 
% INPUT
%   euler: Nx3 Euler angles [roll pitch yaw] (rad, rad, rad).
%
% OUTPUT
%   DCMnb: Nx9 matrix with nav-to-body direct cosine matrices (DCM).
% Each row of DCMnb_m contains the 9 elements of a particular DCMnb
% matrix ordered as [a11 a21 a31 a12 a22 a32 a13 a23 a33].
%
% Reference: 
%
%	Titterton, D.H. and Weston, J.L. (2004). Strapdown
% Inertial Navigation Technology (2nd Ed.). Institution
% of Engineering and Technology, USA. Eq. 3.47, p. 41.
%
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 
    % Roll
    phi = euler(:, 1); 
    % Pitch
    theta = euler(:, 2);
    % Yaw
    psi = euler(:, 3);
    
    [N,~] = size (euler);
    DCMnb_m = zeros(N,9);
    
    for i=1:N
    
        C1 = [cos(psi(i))  sin(psi(i)) 0; ...
            -sin(psi(i)) cos(psi(i)) 0; ...
             0     0   1];
         
        C2 = [cos(theta(i))  0  -sin(theta(i)); ...
              0   1     0 ; ...
            sin(theta(i))  0   cos(theta(i))];
        
        C3 = [1   0    0;   ...
            0  cos(phi(i)) sin(phi(i)); ...
            0 -sin(phi(i)) cos(phi(i))];  
        
        DCMnb = C3 * (C2 * C1);
        DCMnb_m(i,:) = reshape(DCMnb, 1, 9);
    end

end
