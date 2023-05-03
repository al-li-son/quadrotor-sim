function quadrotor_sim

g = 9.81;   %m/s^2
M = .5; %kg
m = .1; %kg
m_total = 4*m + M;
l = .25;  %m

I = [2*m*l^2 0 0; 0 2*m*l^2 0; 0 0 4*m*l^2];

T1 = [0; 0; 4]; %N
T2 = [0; 0; 4]; %N
T3 = [0; 0; 4]; %N
T4 = [0; 0; 3.9]; %N

yaw = sym('yaw');
pitch = sym('pitch');
roll = sym('roll');

% Transform between inertial and body frame
R_psi = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
R_theta = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
R_phi = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];

% Rotate inertial to body
R_321 = R_phi * R_theta * R_psi;

% Rotate body to inertial
R_321_inv = simplify(inv(R_321));

% Euler angle rate equations
% Euler angle derivatives to angular vels
R_euler_omega = [-sin(pitch) 0 1; cos(pitch)*sin(roll) cos(roll) 0; cos(pitch)*cos(roll) -sin(roll) 0];

% Angular vels to euler angle derivatives
R_omega_euler = simplify(inv(R_euler_omega));

% Positions of motors in body frame
r1 = [-l/sqrt(2); -l/sqrt(2); 0];
r2 = [-l/sqrt(2); l/sqrt(2); 0];
r3 = [l/sqrt(2); l/sqrt(2); 0];
r4 = [l/sqrt(2); -l/sqrt(2); 0];

% Simulation parameters
t_max = 3;
r_0 = [0 0 1];
v_0 = [0 0 0];
euler_0 = [0 0 0];
omega_0 = [0 0 0];
Z_0 = [r_0 v_0 euler_0 omega_0];

% ODE45
options = odeset('Events', @event_stop);
[t_out, z_out] = ode45(@quadrotor_fun, [0 t_max], Z_0, options);

x = z_out(:,1);
y = z_out(:,2);
z = z_out(:,3);

m1_coords = zeros(length(z_out),3);
m2_coords = zeros(length(z_out),3);
m3_coords = zeros(length(z_out),3);
m4_coords = zeros(length(z_out),3);

for i=1:length(z_out)
    psi = z_out(i,7);
    theta = z_out(i,8);
    phi = z_out(i,9);
    m1_coords(i,:) = ([x(i); y(i); z(i)] + double(subs(R_321_inv, [yaw, pitch, roll], [psi, theta, phi])) * r1)';
    m2_coords(i,:) = ([x(i); y(i); z(i)] + double(subs(R_321_inv, [yaw, pitch, roll], [psi, theta, phi])) * r2)';
    m3_coords(i,:) = ([x(i); y(i); z(i)] + double(subs(R_321_inv, [yaw, pitch, roll], [psi, theta, phi])) * r3)';
    m4_coords(i,:) = ([x(i); y(i); z(i)] + double(subs(R_321_inv, [yaw, pitch, roll], [psi, theta, phi])) * r4)';
end
figure(1)
plot3(z_out(:,1), z_out(:,2), z_out(:,3), '-o', 'MarkerSize', 3)
hold
for i=1:length(z_out)
    plot3(m1_coords(i,1), m1_coords(i,2), m1_coords(i,3), '.')
    plot3(m2_coords(i,1), m2_coords(i,2), m2_coords(i,3), '.')
    plot3(m3_coords(i,1), m3_coords(i,2), m3_coords(i,3), '.')
    plot3(m4_coords(i,1), m4_coords(i,2), m4_coords(i,3), '.')
    plot3([m1_coords(i,1) m3_coords(i,1)], [m1_coords(i,2) m3_coords(i,2)], [m1_coords(i,3) m3_coords(i,3)], 'k')
    plot3([m2_coords(i,1) m4_coords(i,1)], [m2_coords(i,2) m4_coords(i,2)], [m2_coords(i,3) m4_coords(i,3)], 'k')
    pause(0.1)
end
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
title('Quadcopter Trajectory')
annotation('textbox',[.1 .1 .1 .1],'String',['Thrusts: ', num2str(T1(3)), 'N, ', ...
    num2str(T2(3)), 'N, ', num2str(T3(3)), 'N, ', num2str(T4(3)), 'N'])
axis image

figure(2)
%subplot(4,1,1)
plot(t_out, sqrt(z_out(:,4).^2 + z_out(:,5).^2 + z_out(:,6).^2))
xlabel('Time (s)')
ylabel('Velocity (m/s)')
% subplot(4,1,2)
% plot(t_out, z_out(:,4))
% xlabel('Time (s)')
% ylabel('X Velocity (m/s)')
% subplot(4,1,3)
% plot(t_out, z_out(:,5))
% xlabel('Time (s)')
% ylabel('Y Velocity (m/s)')
% subplot(4,1,4)
% plot(t_out, z_out(:,6))
% xlabel('Time (s)')
% ylabel('Z Velocity (m/s)')

    function dstatedt = quadrotor_fun(T, Z)
        % Z(1-3): x,y,z  Z(4-6): vx,vy,vz
        % Z(7-9): psi,theta,phi  Z(10-12): psi_dot, theta_dot, phi_dot
        vals = num2cell(Z);

        [~,~,~,vx,vy,vz,psi,theta,phi,omega1,omega2,omega3] = deal(vals{:});

        dxdt = vx;
        dydt = vy;
        dzdt = vz;

        euler_dots = subs(R_omega_euler, [yaw, pitch, roll], [psi, theta, phi]) * [omega1; omega2; omega3];
        dpsidt = double(euler_dots(1));
        dthetadt = double(euler_dots(2));
        dphidt = double(euler_dots(3));

        sigma_M = sum_moment(psi, theta, phi);

        domega1dt = double(((I(2,2) - I(3,3))*omega2*omega3 + sigma_M(1))/I(1,1));
        domega2dt = double(((I(3,3) - I(1,1))*omega3*omega1 + sigma_M(2))/I(2,2));
        domega3dt = double(((I(1,1) - I(2,2))*omega1*omega2 + sigma_M(3))/I(3,3));

        accel = (1/m_total) * (subs(R_321_inv, [yaw, pitch, roll], [psi, theta, phi]) * (T1+T2+T3+T4)) - [0; 0; g];

        dvxdt = double(accel(1));
        dvydt = double(accel(2));
        dvzdt = double(accel(3));

        dstatedt = [dxdt; dydt; dzdt; dvxdt; dvydt; dvzdt; dpsidt; dthetadt; dphidt; domega1dt; domega2dt; domega3dt];
    end

    function [eventvalue,stopthecalc,eventdir] = event_stop(T,Z)
        % stop when z = 0 (hits the ground)
        eventvalue  = Z(3);  % end when z = 0
        stopthecalc = 1;     % stop if event occurs
        eventdir    = -1;    % direction conditon = dydt<0
    end

    function res = sum_moment(y, p, r)
        k = subs(R_321_inv(3,:),[yaw, pitch, roll], [y, p, r]);
        res = cross(r1, T1) + cross(r2, T2) + cross(r3, T3) + cross(r4, T4) ...
            + cross(r1, -m*g*k) + cross(r2, -m*g*k) + cross(r3, -m*g*k) + ...
            cross(r4, -m*g*k);
    end

end