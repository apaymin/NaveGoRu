function DCMnb_m = euler2dcm_bn_m(euler)
% euler2dcm: converts from Euler angles to DCM body-to-nav.
% 
% INPUT
%   euler: Nx3 Euler angles [roll pitch yaw] (rad, rad, rad).
%
% OUTPUT
%   DCMnb: Nx9 matrix with body-to-nav direct cosine matrices (DCM).
% Each row of DCMnb_m contains the 9 elements of a particular DCMnb
% matrix ordered as [a11 a21 a31 a12 a22 a32 a13 a23 a33].
%
% Reference: 
%
% M. S. Grewal, L. R. Weill, and A. P. Andrews (2007). Global Positioning Systems, 
% Inertial Navigation, and Integration, Second Edition. P. 478, eq (C.106).
%
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

    phi = euler(:, 1); theta = euler(:, 2); psi = euler(:, 3);
    
    [N,~] = size (euler);
    DCMnb_m = zeros(N,9);
    
    for i=1:N
        % Почему-то формула сбивается
        % C1 = [cos(psi(i))  -sin(psi(i)) 0; ...
        %     sin(psi(i)) cos(psi(i)) 0; ...
        %      0     0   1];
        % 
        % C2 = [cos(theta(i))  0  sin(theta(i)); ...
        %       0   1     0 ; ...
        %     -sin(theta(i))  0   cos(theta(i))];
        % 
        % C3 = [1   0    0;   ...
        %     0  cos(phi(i)) -sin(phi(i)); ...
        %     0 sin(phi(i)) cos(phi(i))]; 

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
        % DCMnb_m(i,:) = reshape(DCMnb, 1, 9);
        DCMnb_m(i,:) = reshape(inv(DCMnb), 1, 9);
    end

end
