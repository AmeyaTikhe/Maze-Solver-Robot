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
#define BLACK_BOUNDARY 945 // Boundary value to distinguish between black and white readings

/*
 * weights given to respective line sensor
 */
const int weights[5] = {-5, -3, 1, 3, 5};

/*
 * Motor value boundts
 */
int optimum_duty_cycle = 57;
int lower_duty_cycle = 45;
int higher_duty_cycle = 65;
float left_duty_cycle = 0, right_duty_cycle = 0;
// Left = 1
// Plus = 2
// Right = 3
// Dead End = 4
int turns_tracker[100];
int turn_number = 0;
int left_indicator = 0, right_indicator = 0, u_turn_indicator = 0, junction_indicator = 0;
int period = 50;

/*
 * Line Following PID Variables
 */
float error = 0, prev_error = 0, difference, cumulative_error, correction;

/*
 * Union containing line sensor readings
 */
line_sensor_array line_sensor_readings;

void calculate_correction()
{
    error = error * 10; // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error;
    cumulative_error += error;

    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp * error + read_pid_const().ki * cumulative_error + read_pid_const().kd * difference;
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1; // assuming initially all black condition
    float weighted_sum = 0, sum = 0;
    float pos = 0;
    int k = 0;

    for (int i = 0; i < 5; i++)
    {
        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
        {
            all_black_flag = 0;
        }
        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
        {
            k = 1;
        }
        if (line_sensor_readings.adc_reading[i] < BLACK_BOUNDARY)
        {
            k = 0;
        }
        weighted_sum += (float)(weights[i]) * k;
        sum = sum + k;
    }

    if (sum != 0) // sum can never be 0 but just for safety purposes
    {
        pos = (weighted_sum - 1) / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
    }

    if (all_black_flag == 1) // If all black then we check for previous error to assign current error.
    {
        if (prev_error > 0)
        {
            error = 2.5;
        }
        else
        {
            error = -2.5;
        }
    }
    else
    {
        error = pos;
    }
}

void calculate_color_blind_error()
{
    int all_white_flag = 1; // assuming initially all white condition
    float weighted_sum = 0, sum = 0;
    float pos = 0;
    int k = 0;

    for (int i = 0; i < 5; i++)
    {
        if (line_sensor_readings.adc_reading[i] < BLACK_BOUNDARY)
        {
            all_white_flag = 0;
        }
        if (line_sensor_readings.adc_reading[i] < BLACK_BOUNDARY)
        {
            k = 1;
        }
        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
        {
            k = 0;
        }
        weighted_sum += (float)(weights[i]) * k;
        sum = sum + k;
    }

    if (sum != 0) // sum can never be 0 but just for safety purposes
    {
        pos = (weighted_sum - 1) / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
    }

    if (all_white_flag == 1) // If all black then we check for previous error to assign current error.
    {
        if (prev_error > 0)
        {
            error = 2.5;
        }
        else
        {
            error = -2.5;
        }
    }
    else
    {
        error = pos;
    }
}

void anticlockwise_rot(motor_handle_t motor_a_0, motor_handle_t motor_a_1)
{

    bool left = false;
    while (1)
    {
        if (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY)
        {
            left = true;
        }
        set_motor_speed(motor_a_0, MOTOR_FORWARD, 45);
        set_motor_speed(motor_a_1, MOTOR_BACKWARD, 45);
        if (line_sensor_readings.adc_reading[0] < BLACK_BOUNDARY && line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[3] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[4] < BLACK_BOUNDARY && left)
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            left = false;
            set_motor_speed(motor_a_0, MOTOR_STOP, 0);
            set_motor_speed(motor_a_1, MOTOR_STOP, 0);
            break;
        }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void clockwise_rot(motor_handle_t motor_a_0, motor_handle_t motor_a_1)
{
    bool right = false;
    while (1)
    {
        if (line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY)
        {
            right = true;
        }
        set_motor_speed(motor_a_1, MOTOR_FORWARD, 45);
        set_motor_speed(motor_a_0, MOTOR_BACKWARD, 45);
        if (line_sensor_readings.adc_reading[0] < BLACK_BOUNDARY && line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[3] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[4] < BLACK_BOUNDARY && right)
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            right = false;
            set_motor_speed(motor_a_0, MOTOR_STOP, 0);
            set_motor_speed(motor_a_1, MOTOR_STOP, 0);
            break;
        }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void line_follow_task(void *arg)
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

    int left_indicator = 0, right_indicator = 0, u_turn_indicator = 0, junction_indicator = 0;

    while (true)
    {
        line_sensor_readings = read_line_sensor(line_sensor);
        for (int i = 0; i < 5; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
        }
        // ESP_LOGI("debug", "LSA_1: %d \t LSA_2: %d \t LSA_3: %d \t LSA_4: %d \t LSA_5: %d", line_sensor_readings.adc_reading[0], line_sensor_readings.adc_reading[1], line_sensor_readings.adc_reading[2], line_sensor_readings.adc_reading[3], line_sensor_readings.adc_reading[4]);
        // colour-blind condition
        if (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[1] < BLACK_BOUNDARY && line_sensor_readings.adc_reading[2] < BLACK_BOUNDARY && line_sensor_readings.adc_reading[3] < BLACK_BOUNDARY && line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY)
        {
            calculate_color_blind_error();
        }
        else
        {
            calculate_error();
        }
        calculate_correction();

        // Increment indicators based on line sensor readings
        if (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY)
        {
            left_indicator++;
        }
        else if (line_sensor_readings.adc_reading[0] < BLACK_BOUNDARY && line_sensor_readings.adc_reading[1] < BLACK_BOUNDARY)
        {
            left_indicator = 0;
            right_indicator = 0;
            u_turn_indicator = 0;
            junction_indicator = 0;
        }
        else if (line_sensor_readings.adc_reading[2] < BLACK_BOUNDARY)
        {
            u_turn_indicator++;
        }
        else if (line_sensor_readings.adc_reading[2] < BLACK_BOUNDARY)
        {
            left_indicator = 0;
            right_indicator = 0;
            u_turn_indicator = 0;
            junction_indicator = 0;
        }
        else
        {
            right_indicator++;
        }

        // Perform the corresponding turn when the indicator reaches threshold
        if (u_turn_indicator >= 18)
        {
            set_motor_speed(motor_a_0, MOTOR_STOP, 0);
            set_motor_speed(motor_a_1, MOTOR_STOP, 0);
            turns_tracker[turn_number++] = 4;

            anticlockwise_rot(motor_a_0, motor_a_1);
            ESP_LOGI("action", "uturn started");
            u_turn_indicator = 0;
        }
        else if (left_indicator >= 18)
        {
            set_motor_speed(motor_a_0, MOTOR_STOP, 0);
            set_motor_speed(motor_a_1, MOTOR_STOP, 0);
            turns_tracker[turn_number++] = 1;
            anticlockwise_rot(motor_a_0, motor_a_1);
            ESP_LOGI("action", "left_turn started");
            left_indicator = 0;
        }
        else if (right_indicator >= 18)
        {
            set_motor_speed(motor_a_0, MOTOR_STOP, 0);
            set_motor_speed(motor_a_1, MOTOR_STOP, 0);
            turns_tracker[turn_number++] = 3;
            ESP_LOGI("action", "right_turn started");
            clockwise_rot(motor_a_0, motor_a_1);
            right_indicator = 0;
        }
        else
        {
            // Line following control logic
            left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
            right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);

            set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}