function [llh] = gost2llh (gost, llh_org)
% body2llh: generates positions in the navigation frame from
% positions in the body frame.
%
% INPUT
%       gost: Nx3 coordinates in GOST 20058-80 navigational frame
%             [X Y Z] (m, m, m)
%		llh_org: 1x3 system origin [lat, lon, h] (rad, rad, m).
%
% OUTPUT
%       llh: Nx3 LLH coordinates [lat, lon, h] (rad, rad, m).
    
    % Перевод координат из ГОСТа в NED
    % По ГОСТ x направлен вдоль строительной оси, y противоположно вектору
    %   ускорения свободного падения, z дополняет до правой тройки
    % В NED x направлен вдоль строительной оси, z сонаправлен вектору
    %   ускорения свободного падения, н дополняет до правой тройки
    ned = [gost(:, 1) gost(:,3) -gost(:, 2)];
    
    % Начало координат в ECEF
    ecef_org = llh2ecef(llh_org);
    % Переход из навигационной NED в географическую ECEF
    ecef = ned2ecef(ned, llh_org) + ecef_org;

    % Переход из географической прямоугольной ECEF в геодезическую LLH
    llh = ecef2llh(ecef);
end
