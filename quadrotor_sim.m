function quadrotor_sim(T,r_0,v_0,euler_0,omega_0)
%% CONSTANTS
g = 9.81;   %m/s^2
M = 22/1000; %kg
m = 2/1000; %kg
m_total = 4*m + M;
l = .05;  %m
I = [2*m*l^2 0 0; 0 2*m*l^2 0; 0 0 4*m*l^2];

% Positions of motors in body frame
r1 = [-l/sqrt(2); -l/sqrt(2); 0];
r2 = [-l/sqrt(2); l/sqrt(2); 0];
r3 = [l/sqrt(2); l/sqrt(2); 0];
r4 = [l/sqrt(2); -l/sqrt(2); 0];

%% INITIAL CONDITIONS
T1 = [0; 0; T(1)]; %N
T2 = [0; 0; T(2)]; %N
T3 = [0; 0; T(3)]; %N
T4 = [0; 0; T(4)]; %N

%% MATH FOR TRANSFORMS
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

%% SIMULATION
% Simulation parameters
t_max = 3;
Z_0 = [r_0 v_0 euler_0 omega_0];

% ODE45
options = odeset('Events', @event_stop);
[t_out, z_out] = ode45(@quadrotor_fun, [0 t_max], Z_0, options);

%% PLOTTING
v_x = z_out(:,4);
v_y = z_out(:,5);
v_z = z_out(:,6);
v_abs = sqrt(v_x.^2 + v_y.^2 + v_z.^2);
figure(1);
plot(t_out, v_abs)
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Quadcopter Absolute Velocity')

a_x = diff(v_x)./diff(t_out);
a_y = diff(v_y)./diff(t_out);
a_z = diff(v_z)./diff(t_out);

figure(2);
plot(t_out(2:end), sqrt(a_x.^2 + a_y.^2 + a_z.^2))
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Quadcopter Absolute Acceleration')

figure(3);
subplot(3,1,1)
plot(t_out, sign(z_out(:,7)).*mod(abs(z_out(:,7)), 2*pi))
xlabel('Time(s)')
ylabel('Yaw (\psi) Angle (rad)')
subplot(3,1,2)
plot(t_out, sign(z_out(:,8)).*mod(abs(z_out(:,8)), 2*pi))
xlabel('Time(s)')
ylabel('Pitch (\theta) Angle (rad)')
subplot(3,1,3)
plot(t_out, sign(z_out(:,9)).*mod(abs(z_out(:,9)), 2*pi))
xlabel('Time(s)')
ylabel('Roll (\phi) Angle (rad)')
sgtitle('Quadcopter Angular Position')

figure(4);
subplot(3,1,1)
plot(t_out, z_out(:,10))
xlabel('Time(s)')
ylabel('\omega_1 (rad/s)')
subplot(3,1,2)
plot(t_out, z_out(:,11))
xlabel('Time(s)')
ylabel('\omega_2 (rad/s)')
subplot(3,1,3)
plot(t_out, z_out(:,12))
xlabel('Time(s)')
ylabel('\omega_3 (rad/s)')
sgtitle('Quadcopter Angular Velocity (in body frame)')

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

figure(5);
plot3(z_out(:,1), z_out(:,2), z_out(:,3), '-o', 'MarkerSize', 3)
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
title('Quadcopter Trajectory')
pause(0.1)
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
axis image
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