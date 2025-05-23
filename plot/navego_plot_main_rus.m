function  navego_plot_main (ref, gnss, nav_e, gnss_i, nav_i, ref_g, ref_n, ins_gnss_string, ...
    gnss_string, OUTAGE, times_out)
% navego_plot: plots results from ИНС/ГНСС integration dataset.
%
% INPUT
%   ref,    reference dataset.
%   gnss,   GNSS dataset.
%   nav_e,  ИНС/ГНСС integration dataset.
%   gnss_i, GNSS dataset interpolated by reference time vector.
%   nav_i,  ИНС/ГНСС dataset interpolated by reference time vector.
%   ref_n,  reference dataset adjusted by ИНС/ГНСС interpolation.
%   ref_g,  reference dataset interpolated by GNSS interpolation.
%   OUTAGE, 'ON' or 'OFF' (string).
%   times_out, Tx1 vector with outage times where times_out(1) is the start
%     time of 1st Отказ ГНСС, times_out(2) is the end of 1st Отказ ГНСС,
%     times_out(3) is the start of the 2nd Отказ ГНСС, and so on. T should
%     be even.
%
% OUTPUT
%   Several figures.
%
%   Copyright (C) 2014, Rodrigo Gonzalez, all rights reserved.
% 
%   This file is part of NaveGo, an open-source MATLAB toolbox for
%   simulation of integrated navigation systems.
%
%   NaveGo is free software: you can redistribute it and/or modify
%   it under the terms of the GNU Lesser General Public License (LGPL)
%   version 3 as published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU Lesser General Public License for more details.
%
%   You should have received a copy of the GNU Lesser General Public
%   License along with this program. If not, see
%   <http://www.gnu.org/licenses/>.
%
% Version: 015
% Date:    2021/12/14
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

if (nargin < 10)
    
    OUTAGE = 'OFF' ;
    times_out = [];
end

if (~exist('ins_gnss_string','var')), ins_gnss_string = 'ИНС/ГНСС'; end
if (~exist('gnss_string','var')),     gnss_string = 'ГНСС'; end

R2D = (180/pi);     % radians to degrees

plot_parameters;

% Standard deviation * 3 vector from Kalman filter a posteriori covariance matrix
sig3_v = abs(nav_e.Pp(:, 1:16:end).^(0.5)) .* 3; %  Taking only diagonal elements from Pp

% Траектория движения (3D)
figure;
plot3(ref_n.lon.*R2D, ref_n.lat.*R2D, ref_n.h, '--k', LineWidth=3);
hold on
plot3(nav_i.lon.*R2D, nav_i.lat.*R2D, nav_i.h, '-', 'Color', blue, 'LineWidth', 3);
% plot3(ref_n.lon(1).*R2D, ref_n.lat(1).*R2D, ref_n.h(1), 'or', 'MarkerSize', ms, 'LineWidth', lw);

if (strcmp(OUTAGE,'ON'))
    ref_o = gnss_outage(nav_i, times_out, 'GET');
    plot3(ref_o.lon.*R2D, ref_o.lat.*R2D, ref_o.h, '.r', 'MarkerSize', ms, 'LineWidth', 4) 
    
    % l1 = legend('Референс', ins_gnss_string, 'Отказ ГНСС', 'Позиция', 'SouthEast');
    l1 = legend('Референс', ins_gnss_string, 'Отказ ГНСС');
else
    % l1 = legend('Референс', ins_gnss_string, 'Позиция', 'SouthEast');
    l1 = legend('Референс', ins_gnss_string);
end

hold off

axis tight
t1 = title('Траектория движения (3D)');
x1 = xlabel('Долгота, °');
y1 = ylabel('Широта, °');
z1 = zlabel('Высота, м');
view(45, 45)
grid

set(t1,'FontSize', font_title);
set(x1,'FontSize', font_label);
set(y1,'FontSize', font_label);
set(z1,'FontSize', font_label);
set(l1,'FontSize', font_legend);
set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);

% Траектория движения (проекция)
figure;
plot(ref.lon.*R2D, ref.lat.*R2D, '--k', LineWidth=2);
hold on
plot(nav_i.lon.*R2D, nav_i.lat.*R2D, 'Color', blue, 'LineWidth', lw);
% plot(ref.lon(1).*R2D, ref.lat(1).*R2D, 'or', 'MarkerSize', ms, 'LineWidth', lw)

if (strcmp(OUTAGE,'ON'))
    ref_o = gnss_outage(nav_i, times_out, 'GET');
    plot(ref_o.lon.*R2D, ref_o.lat.*R2D, '.r', 'MarkerSize', ms, 'LineWidth', 4) 
    
    % l1 = legend('Референс', ins_gnss_string, 'Отказ ГНСС', 'Позиция', 'SouthEast');
    l1 = legend('Референс', ins_gnss_string, 'Отказ ГНСС');
else
    % l1 = legend('Референс', ins_gnss_string, 'Позиция', 'SouthEast');
    l1 = legend('Референс', ins_gnss_string);
end
hold off

% axis tight
t1 = title('Траектория движения (проекция)');
x1 = xlabel('Долгота, °');
y1 = ylabel('Широта, °');

grid

set(t1,'FontSize', font_title);
set(x1,'FontSize', font_label);
set(y1,'FontSize', font_label);
set(l1,'FontSize', font_legend);
set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);

% ATTITUDE
figure;

subplot(311)
cell_s = {'Крен, °', 'Время, с', '[град.]', 'Референс', ins_gnss_string};
navego_plot_thick(cell_s, 'NORMAL', ref.t,  R2D.*ref.roll, nav_e.t, R2D.*nav_e.roll);
    navego_plot_outage (nav_e.t, R2D.*nav_e.roll, OUTAGE, times_out);

subplot(312)
cell_s = {'Тангаж, °', 'Время, с', '[град.]', 'Референс', ins_gnss_string};
navego_plot_thick(cell_s, 'NORMAL', ref.t,  R2D.*ref.pitch, nav_e.t, R2D.*nav_e.pitch);
navego_plot_outage (nav_e.t, R2D.*nav_e.pitch, OUTAGE, times_out);

subplot(313)
cell_s = {'Рысканье, °', 'Время, с', '[град.]', 'Референс', ins_gnss_string};
navego_plot_thick(cell_s, 'NORMAL', ref.t,  R2D.*ref.yaw, nav_e.t, R2D.*nav_e.yaw);
navego_plot_outage (nav_e.t, R2D.*nav_e.yaw, OUTAGE, times_out);

% ATTITUDE ERRORS
figure;

subplot(311)
cell_s = {'Ошибка определения крена, °', 'Время, с', '[град.]', '3\sigma', ins_gnss_string};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,1), nav_i.t, R2D.*(nav_i.roll - ref_n.roll));
navego_plot_outage (nav_i.t, R2D.*(nav_i.roll - ref_n.roll), OUTAGE, times_out);

subplot(312)
cell_s = {'Ошибка определения тангажа, °', 'Время, с', '[град.]', '3\sigma', ins_gnss_string};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,2), nav_i.t, R2D.*(nav_i.pitch - ref_n.pitch));
navego_plot_outage (nav_i.t, R2D.*(nav_i.pitch - ref_n.pitch), OUTAGE, times_out);

subplot(313)
cell_s = {'Ошибка определения рысканья, °', 'Время, с', '[град.]', '3\sigma', ins_gnss_string};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,3), nav_i.t, R2D.*(nav_i.yaw - ref_n.yaw));
navego_plot_outage (nav_i.t, R2D.*(nav_i.yaw - ref_n.yaw), OUTAGE, times_out);

% VELOCITIES
figure;

subplot(311)
cell_s = {'v_N, м/с', 'Время, с', 'м/c', 'Референс', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'NORMAL', ref.t, ref.vel(:,1), nav_e.t, nav_e.vel(:,1), gnss.t, gnss.vel(:,1));
navego_plot_outage (nav_e.t, nav_e.vel(:,1), OUTAGE, times_out);

subplot(312)
cell_s = {'v_E, м/с', 'Время, с', 'м/c', 'Референс', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'NORMAL', ref.t, ref.vel(:,2), nav_e.t, nav_e.vel(:,2), gnss.t, gnss.vel(:,2));
navego_plot_outage (nav_e.t, nav_e.vel(:,2), OUTAGE, times_out);

subplot(313)
cell_s = {'v_D, м/с', 'Время, с', 'м/c', 'Референс', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'NORMAL', ref.t, ref.vel(:,3), nav_e.t, nav_e.vel(:,3), gnss.t, gnss.vel(:,3));
navego_plot_outage (nav_e.t, nav_e.vel(:,3), OUTAGE, times_out);

% VELOCITIES ERRORS
figure;

subplot(311)
cell_s = {'Ошибка определения v_N, м/с', 'Время, с', 'м/c', '3\sigma', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, sig3_v(:,4), ...
    nav_i.t, (nav_i.vel(:,1) - ref_n.vel(:,1)), ...
    gnss_i.t, (gnss_i.vel(:,1) - ref_g.vel(:,1)) ) ;
navego_plot_outage (nav_i.t, (nav_i.vel(:,1) - ref_n.vel(:,1)), OUTAGE, times_out);

subplot(312)
cell_s = {'Ошибка определения v_E, м/с', 'Время, с', 'м/c', '3\sigma', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, sig3_v(:,5), ...
    nav_i.t, (nav_i.vel(:,2) - ref_n.vel(:,2)), ...
    gnss_i.t, (gnss_i.vel(:,2) - ref_g.vel(:,2)) ) ;
navego_plot_outage (nav_i.t, (nav_i.vel(:,2) - ref_n.vel(:,2)), OUTAGE, times_out);

subplot(313)
cell_s = {'Ошибка определения v_D, м/с', 'Время, с', 'м/c', '3\sigma', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, sig3_v(:,6), ...
        nav_i.t, (nav_i.vel(:,3) - ref_n.vel(:,3)), ...
    gnss_i.t, (gnss_i.vel(:,3) - ref_g.vel(:,3)) ) ;
navego_plot_outage (nav_i.t, (nav_i.vel(:,3) - ref_n.vel(:,3)), OUTAGE, times_out);

% POSITION
figure;

subplot(311)
cell_s = {'Широта, °', 'Время, с', '[град.]', 'Референс', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'NORMAL', ref.t, R2D.*ref.lat, nav_e.t, R2D.*nav_e.lat, gnss.t, R2D.*gnss.lat);
navego_plot_outage (nav_e.t, R2D.*nav_e.lat, OUTAGE, times_out);

subplot(312)
cell_s = {'Долгота, °', 'Время, с', '[град.]', 'Референс', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'NORMAL', ref.t, R2D.*ref.lon, nav_e.t, R2D.*nav_e.lon, gnss.t, R2D.*gnss.lon);
navego_plot_outage (nav_e.t, R2D.*nav_e.lon, OUTAGE, times_out);

subplot(313)
cell_s = {'Высота, м', 'Время, с', '[м]', 'Референс', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'NORMAL', ref.t, ref.h, nav_e.t, nav_e.h, gnss.t, gnss.h);
navego_plot_outage (nav_e.t, nav_e.h, OUTAGE, times_out);

% POSITION ERRORS
[RN,RE]  = radius(nav_i.lat);
LAT2M_N = RN + nav_i.h;                     % Radians to meters
LON2M_N = (RE + nav_i.h).*cos(nav_i.lat);   % Radians to meters

[RN,RE]  = radius(gnss.lat);
LAT2M_G = RN + gnss.h;                      % Radians to meters
LON2M_G = (RE + gnss.h).*cos(gnss.lat);     % Radians to meters

[RN,RE]  = radius(gnss_i.lat);
LAT2M_I = RN + gnss_i.h;                    % Radians to meters
LON2M_I = (RE + gnss_i.h).*cos(gnss_i.lat); % Radians to meters

figure;

subplot(311)
cell_s = {'Ошибка определения широты, м', 'Время, с', 'м', '3\sigma', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, LAT2M_G .* sig3_v(:,7), ...
                             nav_i.t,  LAT2M_N .* (nav_i.lat - ref_n.lat), ... 
                             gnss_i.t, LAT2M_I .* (gnss_i.lat - ref_g.lat) );
navego_plot_outage (nav_i.t,  LAT2M_N .* (nav_i.lat - ref_n.lat), OUTAGE, times_out);

subplot(312)
cell_s = {'Ошибка определения долготы, м', 'Время, с', 'м', '3\sigma', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, LON2M_G .* sig3_v(:,8), ...
                             nav_i.t,  LON2M_N .* (nav_i.lon - ref_n.lon) , ...
                             gnss_i.t, LON2M_I .* (gnss_i.lon - ref_g.lon) );
navego_plot_outage (nav_i.t,  LON2M_N .* (nav_i.lon - ref_n.lon), OUTAGE, times_out);

subplot(313)
cell_s = {'Ошибка определения высоты, м', 'Время, с', 'м', '3\sigma', gnss_string, ins_gnss_string};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, sig3_v(:,9), ...
                             nav_i.t, (nav_i.h - ref_n.h) , ...
                             gnss_i.t, (gnss_i.h - ref_g.h) );
navego_plot_outage (nav_i.t,  (nav_i.h - ref_n.h), OUTAGE, times_out);

% BIAS ESTIMATION
figure;

subplot(311)
cell_s = {'KF BIAS GYRO X ESTIMATION', 'Время, с', '[deg/s]', '3\sigma', 'BIAS'};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,10), nav_e.tg, R2D.*nav_e.b(:, 1));
% navego_plot_outage (nav_e.tg, R2D.*nav_e.b(:, 1), OUTAGE, times_out);

subplot(312)
cell_s = {'KF BIAS GYRO Y ESTIMATION', 'Время, с', '[deg/s]', '3\sigma', 'BIAS'};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,11), nav_e.tg, R2D.*nav_e.b(:, 2));
% navego_plot_outage (nav_e.tg, R2D.*nav_e.b(:, 2), OUTAGE, times_out);

subplot(313)
cell_s = {'KF BIAS GYRO Z ESTIMATION', 'Время, с', '[deg/s]', '3\sigma', 'BIAS'};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,12), nav_e.tg, R2D.*nav_e.b(:, 3));
% navego_plot_outage (nav_e.tg, R2D.*nav_e.b(:, 3), OUTAGE, times_out);

figure;

subplot(311)
cell_s = {'KF BIAS ACCR X ESTIMATION', 'Время, с', 'м/c', '3\sigma', 'BIAS'};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, sig3_v(:,13), nav_e.tg, nav_e.b(:, 4));
% navego_plot_outage (nav_e.tg, nav_e.b(:, 4), OUTAGE, times_out);

subplot(312)
cell_s = {'KF BIAS ACCR Y ESTIMATION', 'Время, с', 'м/c', '3\sigma', 'BIAS'};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, sig3_v(:,14), nav_e.tg, nav_e.b(:, 5));
% navego_plot_outage (nav_e.tg, nav_e.b(:, 5), OUTAGE, times_out);

subplot(313)
cell_s = {'KF BIAS ACCR Z ESTIMATION', 'Время, с', 'м/c', '3\sigma', 'BIAS'};
navego_plot_thick(cell_s, 'ERROR', nav_e.tg, sig3_v(:,15), nav_e.tg, nav_e.b(:, 6));
% navego_plot_outage (nav_e.tg, nav_e.b(:, 6), OUTAGE, times_out);

% STATES OBSERVABILITY
figure
cell_s = {'STATES OBSERVABILITY', 'Время, с', 'STATES', 'Референс', ins_gnss_string};
navego_plot_thick(cell_s, 'NORMAL', nav_e.tg(1), 0, nav_e.tg, nav_e.ob);
