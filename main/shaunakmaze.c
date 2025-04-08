#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"

#define MODE NORMAL_MODE
#define BLACK_MARGIN 4095
#define WHITE_MARGIN 0
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define BLACK_BOUNDARY  950
#define TURN_INTERVAL 2000     // 2 seconds interval for 30-degree left turn


const int weights[5] = {-5, -3, 1, 3, 5};

// Motor value bounds
int optimum_duty_cycle = 55;
int lower_duty_cycle = 44;
int higher_duty_cycle = 64;
float left_duty_cycle = 0, right_duty_cycle = 0;

// PID Variables
float error=0, prev_error=0, difference, cumulative_error, correction;
bool left_turn_detected = false;
bool right_turn_detected = false;
bool u_turn_detected = false;

TickType_t last_turn_time = 0;
const TickType_t turn_delay_ticks = pdMS_TO_TICKS(TURN_INTERVAL);
bool turning_left_timer = false;

// Line Sensor Readings
line_sensor_array line_sensor_readings;

void calculate_correction()
{
    difference = error - prev_error;
    cumulative_error += error;
    cumulative_error = bound(cumulative_error, -30, 30);

    correction = 0.9 * error + 0* cumulative_error + 6.5 * difference;
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1;
    float weighted_sum = 0, sum = 0;
    float pos = 0;
    int k = 0;

    for(int i = 0; i < 5; i++)
    {
        if(line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
        {
            all_black_flag = 0;
            k = 1;
        }
        else
        {
            k = 0;
        }
        weighted_sum += (float)(weights[i]) * k;
        sum += k;
    }

    if(sum != 0)
    {
        pos = (weighted_sum - 1) / sum;
    }

    if(all_black_flag == 1)
    {
        if(prev_error > 0)
        {
            error = 8;
        }
        else
        {
            error = -8;
        }
    }
    else
    {
        error = pos;
    }
}

void handle_intersections()
{
    bool left_sensor = line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY;
    bool left_buffer = line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY;
    bool straight_sensor = line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY;
    bool right_buffer = line_sensor_readings.adc_reading[3] > BLACK_BOUNDARY;
    bool right_sensor = line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY;

    int active_sensors = 0;

    for (int i = 0; i < 5; i++)
    {
        if(line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
        {
            active_sensors++;
        }
    }

    if (left_sensor && !right_sensor)
    {
        left_turn_detected = true;
        right_turn_detected = false;
        u_turn_detected = false;
    }
    else if (!left_sensor && !right_sensor)
    {
        left_turn_detected = false;
        right_turn_detected = false;
        u_turn_detected = false;
    }
    else if (right_sensor && !left_sensor)
    {
        left_turn_detected = false;
        right_turn_detected = true;
        u_turn_detected = false;
    }
    else if (!right_sensor && !left_sensor && !straight_sensor)
    {
        left_turn_detected = false;
        right_turn_detected = false;
        u_turn_detected = true;
    }
    else
    {
        left_turn_detected = false;
        right_turn_detected = false;
        u_turn_detected = false;
    }
}

void line_follow_task(void* arg)
{
    motor_handle_t motor_a_0, motor_a_1;
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));
    adc_handle_t line_sensor;
    ESP_ERROR_CHECK(enable_line_sensor(&line_sensor));
    ESP_ERROR_CHECK(enable_bar_graph());

#ifdef CONFIG_ENABLE_OLED
    ESP_ERROR_CHECK(init_oled());
    vTaskDelay(100);
    lv_obj_clean(lv_scr_act());
#endif

    while(true)
    {
        line_sensor_readings = read_line_sensor(line_sensor);
        for(int i = 0; i < 5; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
        }

        // if (xTaskGetTickCount() - last_turn_time >= turn_delay_ticks) {
        //     last_turn_time = xTaskGetTickCount();
        //     turning_left_timer = true;
        // }

        // if (turning_left_timer) {
        //     // 30-degree left turn logic
        //     set_motor_speed(motor_a_0, MOTOR_BACKWARD, 60);
        //     set_motor_speed(motor_a_1, MOTOR_FORWARD, 60);
        //     vTaskDelay(pdMS_TO_TICKS(500));  // Delay to simulate a 30-degree turn
        //     turning_left_timer = false;
        //     last_turn_time = xTaskGetTickCount();
        //     ESP_LOGI("action","Turn complete");
        // }

        calculate_error();
        handle_intersections();
        calculate_correction();

        if (left_turn_detected)
        {
            set_motor_speed(motor_a_0, MOTOR_BACKWARD, 0);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 0);
            ESP_LOGI("action","Left turn");

            vTaskDelay(pdMS_TO_TICKS(50));
            // In-place left turn: left motor backward, right motor forward
            left_duty_cycle = bound(optimum_duty_cycle, lower_duty_cycle, higher_duty_cycle);
            right_duty_cycle = bound(optimum_duty_cycle, lower_duty_cycle, higher_duty_cycle);

            set_motor_speed(motor_a_0, MOTOR_BACKWARD, left_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
        }
        else if (right_turn_detected)
        {
            set_motor_speed(motor_a_0, MOTOR_FORWARD, 0);
            set_motor_speed(motor_a_1, MOTOR_BACKWARD, 0);
            ESP_LOGI("action","Right turn");

            vTaskDelay(pdMS_TO_TICKS(50));
            // In-place right turn: right motor backward, left motor forward
            left_duty_cycle = bound(optimum_duty_cycle, lower_duty_cycle, higher_duty_cycle);
            right_duty_cycle = bound(optimum_duty_cycle, lower_duty_cycle, higher_duty_cycle);

            set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_BACKWARD, right_duty_cycle);
        }
        else if (u_turn_detected)
        {
            set_motor_speed(motor_a_0, MOTOR_BACKWARD, 0);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 0);
            ESP_LOGI("action","U turn");

            vTaskDelay(pdMS_TO_TICKS(50));
            // In-place left turn: left motor backward, right motor forward
            left_duty_cycle = bound(optimum_duty_cycle, lower_duty_cycle, higher_duty_cycle);
            right_duty_cycle = bound(optimum_duty_cycle, lower_duty_cycle, higher_duty_cycle);

            set_motor_speed(motor_a_0, MOTOR_BACKWARD, optimum_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, optimum_duty_cycle);
        }
        else
        {
            // Regular PID-based control for line following
            left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
            right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);

            set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
        }

        ESP_LOGI("debug", "KP: %f :: KI: %f :: KD: %f :: Correction: %f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd, correction);

#ifdef CONFIG_ENABLE_OLED
        if (read_pid_const().val_changed)
        {
            display_pid_values(read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
            reset_val_changed_pid_const();
        }
#endif

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}