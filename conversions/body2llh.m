function [llh] = body2llh (body, euler, llh_org, DCMbn_m)
% body2llh: generates positions in the navigation frame from
% positions in the body frame.
%
% INPUT
%       body: Nx3 coordinates in body frame [X Y Z] (m, m, m)
%       euler: Nx3 Euler angles [roll pitch yaw] (rad, rad, rad).
%		llh_org: 1x3 system origin [lat, lon, h] (rad, rad, m).
%
% OUTPUT
%       llh: Nx3 LLH coordinates [lat, lon, h] (rad, rad, m).
    
    % Матрица перехода из связанной в навигационную СК
    if ~exist('DCMbn_m', 'var')
        DCMbn_m = euler2dcm_bn_m(euler);
    end

    % Переход из связанной в навигационную СК NED
    ned = nav2body(body, DCMbn_m);
    
    % Начало координат в ECEF
    ecef_org = llh2ecef(llh_org);
    % Переход из навигационной NED в географическую ECEF
    ecef = ned2ecef(ned, llh_org) + ecef_org;

    % Переход из географической прямоугольной ECEF в геодезическую LLH
    llh = ecef2llh(ecef);
end
