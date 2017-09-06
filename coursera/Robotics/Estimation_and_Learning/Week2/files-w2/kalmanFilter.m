function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    dt = t - previous_t;
    F_ = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
    noise_ax = 0.1;
    noise_ay = 0.1;
    dt_2 = dt * dt;
    dt_3 = dt_2 * dt;
    dt_4 = dt_3 * dt;
    Q_ = [(dt_4 * noise_ax / 4) 0 (dt_3 * noise_ax / 2) 0;
           0 (dt_4 * noise_ay / 4) 0 (dt_3 * noise_ay / 2);
           (dt_3 * noise_ax / 2) 0 (dt_2 * noise_ax) 0;
           0 (dt_3 * noise_ay / 2) 0 (dt_2 * noise_ay)];

    pred_state = F_ * state';
    P_ = F_ * param.P * F_' + Q_;

    C_ = [1 0 0 0;
          0 1 0 0];

    noise_zx = 0.1;
    noise_zy = 0.1;

    R_ = [noise_zx 0;
          0 noise_zy];

    z_pred = C_ * pred_state;
    z = [x; y]
    y = z - z_pred;

    S = C_ * P_ * C_' + R_;
    Si = inv(S);
    PHt = P_ * C_';

    K = PHt * Si;

    state = pred_state + (K * y);
    param.P = (eye(4) - K * C_) * P_;

    %vx = (x - state(1)) / (t - previous_t);
    %vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
    %predictx = x + vx * 0.330;
    %predicty = y + vy * 0.330;
    % State is a four dimensional element
    %state = [x, y, vx, vy];
    Fx = [1 0 0.33 0];
    Fy = [0 1 0 0.33];

    predictx = Fx * state;
    predicty = Fy * state;
    state = state';
end
