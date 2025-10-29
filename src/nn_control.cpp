#include "neural_network.h"
#include "sensor.hpp"
#include "rc.hpp"

void vector_add(const float* a, const float* b, float* out, int size) {
    for (int i = 0; i < size; i++) {
        out[i] = a[i] + b[i];
    }
}

/**
 * out = a - b
 */
void vector_subtract(const float* a, const float* b, float* out, int size) {
    for (int i = 0; i < size; i++) {
        out[i] = a[i] - b[i];
    }
}

void matmul_vector(const float* mat, const float* vec, float* out, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        out[i] = 0.0f;
        for (int j = 0; j < cols; j++) {
            out[i] += mat[i * cols + j] * vec[j];
        }
    }
}

void nn_control(void) {
    float nn_input[12];
    float nn_output[4];

    // TODO: euler to axis-angle representation

    // p_rate = Roll_rate;
    // q_rate = Pitch_rate;
    // r_rate = Yaw_rate;

    // TODO: convert euler angles to projected gravity vector, and linear velocity + desired position to body frame
    float roll_vector[3] = {Roll_angle, 0.0f, 0.0f};
    float pitch_vector[3] = {0.0f, Pitch_angle, 0.0f};
    float yaw_vector[3] = {0.0f, 0.0f, Yaw_angle};
    float ang_vel[3];
    vector_add(roll_vector, pitch_vector, ang_vel, 3);
    vector_add(ang_vel, yaw_vector, ang_vel, 3);
    float roll_matrix[9] = {
        1, 0, 0,
        0, cosf(Roll_angle), -sinf(Roll_angle),
        0, sinf(Roll_angle), cosf(Roll_angle)
    };
    float pitch_matrix[9] = {
        cosf(Pitch_angle), 0, sinf(Pitch_angle),
        0, 1, 0,
        -sinf(Pitch_angle), 0, cosf(Pitch_angle)
    };
    float yaw_matrix[9] = {
        cosf(Yaw_angle), -sinf(Yaw_angle), 0,
        sinf(Yaw_angle), cosf(Yaw_angle), 0,
        0, 0, 1
    };
    float projected_gravity[3] = {0.0f, 0.0f, -9.81f};
    matmul_vector(roll_matrix, projected_gravity, projected_gravity, 3, 3);
    matmul_vector(pitch_matrix, projected_gravity, projected_gravity, 3, 3);
    matmul_vector(yaw_matrix, projected_gravity, projected_gravity, 3, 3);

    float position_diff[3];
    float mocap_pos_copy[3] = {mocap_pos[0], mocap_pos[1], mocap_pos[2]};
    float pos_setpoint_copy[3] = {pos_setpoint[0], pos_setpoint[1], pos_setpoint[2]};
    vector_subtract(pos_setpoint_copy, mocap_pos_copy, position_diff, 3);
    // project position diff into body frame
    matmul_vector(roll_matrix, position_diff, position_diff, 3, 3);
    matmul_vector(pitch_matrix, position_diff, position_diff, 3, 3);
    matmul_vector(yaw_matrix, position_diff, position_diff, 3, 3);

    float velocity_body[3];
    float mocap_vel_copy[3] = {mocap_vel[0], mocap_vel[1], mocap_vel[2]};
    matmul_vector(roll_matrix, mocap_vel_copy, velocity_body, 3, 3);
    matmul_vector(pitch_matrix, velocity_body, velocity_body, 3, 3);
    matmul_vector(yaw_matrix, velocity_body, velocity_body, 3, 3);

    // Roll_angle  ;
    // Pitch_angle ;
    // Yaw_angle   ;


    // self._robot.data.root_lin_vel_b,
    // self._robot.data.root_ang_vel_b,
    // self._robot.data.projected_gravity_b,
    // desired_pos_b,
    nn_input[0] = velocity_body[0];
    nn_input[1] = velocity_body[1];
    nn_input[2] = velocity_body[2];
    nn_input[3] = ang_vel[0];
    nn_input[4] = ang_vel[1];
    nn_input[5] = ang_vel[2];
    nn_input[6] = projected_gravity[0];
    nn_input[7] = projected_gravity[1];
    nn_input[8] = projected_gravity[2];
    nn_input[9] = position_diff[0];
    nn_input[10] = position_diff[1];
    nn_input[11] = position_diff[2];

    nn_forward(nn_input, nn_output);

    set_duty_fr((nn_output[0]+1.0)/2.0);
    set_duty_rr((nn_output[1]+1.0)/2.0);
    set_duty_rl((nn_output[2]+1.0)/2.0);
    set_duty_fl((nn_output[3]+1.0)/2.0);
}