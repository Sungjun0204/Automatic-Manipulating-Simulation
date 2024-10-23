% M자 모양의 좁은 통로에서 로봇 팔과 점의 임피던스 제어 시뮬레이션 (목적지까지 이동)

clear all; close all; clc;

%% 로봇 파라미터 설정

% 링크 길이 [m]
L1 = 1.0; % 링크 1의 길이
L2 = 1.0; % 링크 2의 길이

% 링크 질량 [kg]
m1 = 1.0; % 링크 1의 질량
m2 = 1.0; % 링크 2의 질량

% 관성 모멘트 [kg*m^2]
I1 = (1/12)*m1*L1^2; % 링크 1의 관성 모멘트
I2 = (1/12)*m2*L2^2; % 링크 2의 관성 모멘트

% 엔드 이펙터에 부착된 장치의 파라미터
m_device = 18; % 장치의 질량 [kg]
w_device = 0.2;    % 너비 [m]
h_device = 0.3;    % 높이 [m]
I_device_cm = (1/12) * m_device * (w_device^2 + h_device^2); % [kg*m^2] 물체의 COM기준 관성 모멘트

% 중력 가속도 [m/s^2]
g = 0;  % 중력의 영향을 받지 않는 시스템(테이블 위에서 진행하므로)

%% 임피던스 제어기 파라미터 설정

% 로봇 팔과 점 사이의 임피던스 파라미터
M_d = diag([1, 1]); % 가상의 질량 행렬
D_d = diag([50, 50]); % 댐핑 행렬
K_d = diag([50, 50]); % 강성 행렬

% 자석 효과를 위한 상수 설정
magnet_force_constant = 1; % 자석 인력 상수

% 거리 임계값 설정
distance_threshold = 0.3; % 로봇 팔과 점 사이의 최대 거리 [m]

%% 시뮬레이션 설정

dt = 0.005; % 시간 간격 [s]
T = 90; % 시뮬레이션 총 시간 [s]
time = 0:dt:T; % 시간 벡터

% 초기 관절 각도 및 속도
start_point = [-1.5; -1.5]; % M자 통로의 왼쪽 하단 시작 지점
q = inverse_kinematics(start_point, L1, L2); % 초기 관절 각도 [rad]
qd = [0; 0]; % 초기 관절 각속도 [rad/s]

% 점의 초기 위치
p = start_point; % 점의 위치 [m]

% 로봇 팔의 목표 위치
end_point = [1.3; -1.3]; % M자 통로의 오른쪽 하단 끝 지점

% M자 통로의 좌표 정의 (폭이 0.3인 좁은 통로)
maze_x = [-1.5, -0.5, 0, 0.5, 1.5];
maze_y_center = [-1.5, -0.5, -1.5, -0.5, -1.5];
maze_width = 0.3; % 통로의 폭

% 데이터 저장용 변수
q_history = zeros(2, length(time));
x_history = zeros(2, length(time));
p_history = zeros(2, length(time));

%% 시각화 설정

figure;
axis equal;
xlim([-2, 2]);
ylim([-2, 2]);
grid on;
hold on;

% M자 통로 그리기
plot_maze(maze_x, maze_y_center, maze_width);

%% 시뮬레이션 루프

for i = 1:length(time)
    % 현재 엔드이펙터 위치 계산
    x = forward_kinematics(q, L1, L2);

    % 자코비안 계산
    J = jacobian(q, L1, L2);

    % 엔드이펙터 속도 계산
    x_dot = J * qd;

    % 점과 로봇 팔의 거리 계산
    distance = norm(x - p);

    % 점의 위치 업데이트 (자석의 인력 효과 추가)
    attraction_force = magnet_force_constant * (x - p); % 자석 효과에 의한 인력 계산
    p = p + attraction_force * dt; % 점의 위치 업데이트

    % 점이 M자 통로 내부에 있는지 확인
    [inside_maze, p_corrected] = check_maze(p, maze_x, maze_y_center, maze_width);
    if inside_maze
        p = p;
    else
        p = p_corrected; % 점은 통로의 벽에 막혀 이동 불가
    end

    % 임피던스 제어 적용
    e_p = p - x; % 로봇 팔과 점 사이의 위치 오차
    e_v = -x_dot; % 로봇 팔의 엔드이펙터 속도 (점은 정지해 있다고 가정)
    F_impedance = M_d * [0; 0] + D_d * e_v + K_d * e_p;

    % 거리 제한 검사
    if distance > distance_threshold
        % 로봇 팔이 점의 위치로 이동하도록 목표 위치 설정
        x_target = p;
    else
        % 로봇 팔은 목표 지점으로 이동
        x_target = end_point;
    end

    % 목표 위치로의 방향 계산
    direction_to_target = x_target - x;
    if norm(direction_to_target) > 1e-6
        direction_to_target = direction_to_target / norm(direction_to_target);
    else
        direction_to_target = [0; 0];
    end

    % 스텝 크기 설정(SCARA의 속도)
    step_size = 0.1;

    % 새로운 목표 위치 계산
    x_desired = x + direction_to_target * step_size;

    % 로봇 팔의 목표 위치에 대한 역기구학 계산
    q_desired = inverse_kinematics(x_desired, L1, L2);

    % 관절 공간에서의 위치 오차
    e_q = q_desired - q;
    e_qd = -qd;

    % PD 제어기의 게인 조정 (임피던스 제어의 영향 증대)
    Kp = diag([100, 100]);
    Kd = diag([30, 30]);

    % 관절 토크 계산
    tau = Kp * e_q + Kd * e_qd + J' * F_impedance;

    % 동역학 방정식 계산 (장치의 관성 추가)
    [M, C, G] = dynamics_matrices(q, qd, m1, m2, L1, L2, I1, I2, m_device, I_device_cm);

    % 관절 가속도 계산
    epsilon = 1e-6; % 작은 값
    M_reg = M + epsilon * eye(size(M)); % M 행렬에 작은 값을 더함
    qdd = M_reg \ (tau - C * qd - G);

    % 상태 업데이트 (오일러 방법)
    qd = qd + qdd * dt;
    q = q + qd * dt;

    % 데이터 저장
    q_history(:, i) = q;
    x_history(:, i) = x;
    p_history(:, i) = p;

    % 시각화 업데이트
    if mod(i, 20) == 0
        cla;
        % M자 통로 그리기
        plot_maze(maze_x, maze_y_center, maze_width); hold on;
        % 로봇 그리기
        draw_robot(q, L1, L2);
        % 점 그리기
        plot(p(1), p(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        % 목표 위치 표시
        plot(end_point(1), end_point(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
        % 설정
        axis equal;
        xlim([-2, 2]);
        ylim([-2, 2]);
        title(sprintf('Time: %.2f s', time(i)));
        grid on;
        drawnow;
    end

    % 종료 조건 확인 (목표 지점 도달)
    if norm(x - end_point) < 0.05
        fprintf('목표 지점에 도달하였습니다.\n');
        break;
    end
end

%% 결과 시각화

% 엔드이펙터 궤적 플롯
figure;
plot(x_history(1, 1:i), x_history(2, 1:i), 'b', 'LineWidth', 2); hold on;
plot(p_history(1, 1:i), p_history(2, 1:i), 'r', 'LineWidth', 2);
plot(end_point(1), end_point(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
% M자 통로 그리기
plot_maze(maze_x, maze_y_center, maze_width);
xlabel('X Position [m]');
ylabel('Y Position [m]');
legend('End Effector Trajectory', 'Point Trajectory', 'Goal Position', 'Maze');
title('Trajectories of MNS and Magnetic Robot');
grid on;
axis equal;

%% 함수 정의

% 순방향 운동학 함수
function x = forward_kinematics(q, L1, L2)
    theta1 = q(1);
    theta2 = q(2);
    x1 = L1 * cos(theta1);
    y1 = L1 * sin(theta1);
    x2 = x1 + L2 * cos(theta1 + theta2);
    y2 = y1 + L2 * sin(theta1 + theta2);
    x = [x2; y2];
end

% 역기구학 함수
function q = inverse_kinematics(x, L1, L2)
    x_end = x(1);
    y_end = x(2);
    cos_theta2 = (x_end^2 + y_end^2 - L1^2 - L2^2) / (2 * L1 * L2);
    % cos_theta2 값의 범위 제한
    cos_theta2 = min(max(cos_theta2, -1), 1);
    sin_theta2 = sqrt(1 - cos_theta2^2);
    theta2 = atan2(sin_theta2, cos_theta2);
    k1 = L1 + L2 * cos_theta2;
    k2 = L2 * sin_theta2;
    theta1 = atan2(y_end, x_end) - atan2(k2, k1);
    q = [theta1; theta2];
end

% 자코비안 계산 함수
function J = jacobian(q, L1, L2)
    theta1 = q(1);
    theta2 = q(2);
    J11 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2);
    J12 = -L2 * sin(theta1 + theta2);
    J21 = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    J22 = L2 * cos(theta1 + theta2);
    J = [J11, J12; J21, J22];
end

% 동역학 행렬 계산 함수 (관절 1과 관절 2에 대한 물체의 관성 추가)
function [M, C, G] = dynamics_matrices(q, qd, m1, m2, L1, L2, I1, I2, m_device, I_device_cm)
    theta1 = q(1);
    theta2 = q(2);
    theta1_dot = qd(1);
    theta2_dot = qd(2);
    
    % 자코비안 계산
    J = jacobian(q, L1, L2);
    
    % 작업 공간의 관성 행렬 (직육면체의 관성)
    I_effector = diag([I_device_cm, I_device_cm]); % [kg*m^2]
    
    % 관절 공간으로 변환된 장치의 관성 행렬
    M_joint_effector = J' * I_effector * J;
    
    % 기존 질량 행렬 계산
    M11 = I1 + I2 + (m1 * (L1^2)) / 4 + m2 * (L1^2 + (L2^2) / 4 + L1 * L2 * cos(theta2));
    M12 = I2 + m2 * ((L2^2) / 4 + (L1 * L2 * cos(theta2)) / 2);
    M21 = M12;
    M22 = I2 + m2 * (L2^2) / 4;
    M_original = [M11, M12; M21, M22];
    
    % 관절 1과 관절 2에 대한 물체의 관성 모멘트 추가
    % 관절 1과 관절 2에 대한 관성 모멘트를 계산
    % 관절 1과 관절 2에 대해 별도로 변환된 관성 모멘트를 합산
    I_device_joint1 = I_device_cm + m_device * norm(forward_kinematics(q, L1, L2))^2; % 관절 1과의 거리
    I_device_joint2 = I_device_cm + m_device * (L2^2); % 관절 2와의 거리
    
    % 질량 행렬에 관성 추가
    M = M_original + I_device_joint1 + I_device_joint2;
    
    % 코리올리 및 원심력 행렬 C 계산
    C11 = -m2 * L1 * L2 * sin(theta2) * theta2_dot / 2;
    C12 = -m2 * L1 * L2 * sin(theta2) * (theta1_dot + theta2_dot) / 2;
    C21 = m2 * L1 * L2 * sin(theta2) * theta1_dot / 2;
    C22 = 0;
    C = [C11, C12; C21, C22];
    
    % 중력 벡터 G (중력 제거)
    G = [0; 0];
end

% 로봇 그리기 함수
function draw_robot(q, L1, L2)
    theta1 = q(1);
    theta2 = q(2);
    % 관절 위치 계산
    x0 = 0; y0 = 0;
    x1 = L1 * cos(theta1);
    y1 = L1 * sin(theta1);
    x2 = x1 + L2 * cos(theta1 + theta2);
    y2 = y1 + L2 * sin(theta1 + theta2);
    % 로봇 그림
    plot([x0, x1], [y0, y1], 'b-', 'LineWidth', 2); hold on;
    plot([x1, x2], [y1, y2], 'b-', 'LineWidth', 2);
    plot(x0, y0, 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    plot(x1, y1, 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    plot(x2, y2, 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
end

% M자 통로 그리기 함수
function plot_maze(maze_x, maze_y_center, maze_width)
    for i = 1:length(maze_x)-1
        % 각 구간의 선분 그리기 (위쪽 경계)
        x_coords = [maze_x(i), maze_x(i+1)];
        y_coords = [maze_y_center(i), maze_y_center(i+1)] + maze_width / 2;
        plot(x_coords, y_coords, 'k-', 'LineWidth', 2); hold on;
        % 각 구간의 선분 그리기 (아래쪽 경계)
        y_coords = [maze_y_center(i), maze_y_center(i+1)] - maze_width / 2;
        plot(x_coords, y_coords, 'k-', 'LineWidth', 2);
    end
end

% 점이 M자 통로 내부에 있는지 확인하는 함수
function [inside_maze, p_corrected] = check_maze(p, maze_x, maze_y_center, maze_width)
    x = p(1);
    y = p(2);
    y_center = interp1(maze_x, maze_y_center, x, 'linear', 'extrap');
    y_upper = y_center + maze_width / 2;
    y_lower = y_center - maze_width / 2;
    if y >= y_lower && y <= y_upper
        inside_maze = true;
        p_corrected = p;
    else
        inside_maze = false;
        % 벽에 막혀서 y 좌표를 수정
        if y > y_upper
            p_corrected = [x; y_upper];
        else
            p_corrected = [x; y_lower];
        end
    end
end
