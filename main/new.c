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
#define BLACK_BOUNDARY  930    // Boundary value to distinguish between black and white readings
#define TURN_INTERVAL 2000     // 2 seconds interval for 30-degree left turn

/*
 * Weights given to respective line sensor
 */
const int weights[5] = {-5, -3, 1, 3, 5};

/*
 * Motor value bounds
 */
int optimum_duty_cycle = 50;
int lower_duty_cycle = 45;
int higher_duty_cycle = 65;
float left_duty_cycle = 0, right_duty_cycle = 0;

/*
 * Line Following PID Variables
 */
float error=0, prev_error=0, difference, cumulative_error, correction;

/*
 * Union containing line sensor readings
 */
line_sensor_array line_sensor_readings;

// Timer variables for periodic left turn
TickType_t last_turn_time = 0;
const TickType_t turn_delay_ticks = pdMS_TO_TICKS(TURN_INTERVAL);
bool turning_left_timer = false;

void calculate_correction()
{
    error = error * 10;  // We need the error correction in range 0-100 so that we can send it directly as duty cycle parameter
    difference = error - prev_error;
    cumulative_error += error;

    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp * error + read_pid_const().ki * cumulative_error + read_pid_const().kd * difference;
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1;  // Assuming initially all black condition
    float weighted_sum = 0, sum = 0; 
    float pos = 0; 
    int k = 0;

    for(int i = 0; i < 5; i++)
    {
        if(line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
        {
            all_black_flag = 0;
        }
        if(line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
        {
            k = 1;
        }
        if(line_sensor_readings.adc_reading[i] < BLACK_BOUNDARY)
        {
            k = 0;
        }
        weighted_sum += (float)(weights[i]) * k;
        sum = sum + k;
    }

    if(sum != 0)  // Sum can never be 0 but just for safety purposes
    {
        pos = (weighted_sum - 1) / sum; // This will give us the position wrt line. If +ve, then bot is facing left; if -ve, the bot is facing right.
    }

    if(all_black_flag == 1)  // If all black, we check for the previous error to assign the current error.
    {
        if(prev_error > 0)
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

void line_follow_task(void* arg)
{
    motor_handle_t motor_a_0, motor_a_1;
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));
    adc_handle_t line_sensor;
    ESP_ERROR_CHECK(enable_line_sensor(&line_sensor));
    ESP_ERROR_CHECK(enable_bar_graph());

#ifdef CONFIG_ENABLE_OLED
    // Initialising the OLED
    ESP_ERROR_CHECK(init_oled());
    vTaskDelay(100);

    // Clearing the screen
    lv_obj_clean(lv_scr_act());
#endif

    last_turn_time = xTaskGetTickCount();  // Initialize the turn timer

    while(true)
    {
        line_sensor_readings = read_line_sensor(line_sensor);
        for(int i = 0; i < 5; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
        }

        // Timed 30-degree left turn every 2 seconds
        if (xTaskGetTickCount() - last_turn_time >= turn_delay_ticks) {
            last_turn_time = xTaskGetTickCount();
            turning_left_timer = true;
        }

        if (turning_left_timer) {
            // 30-degree left turn logic
            set_motor_speed(motor_a_0, MOTOR_BACKWARD, 60);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 60);
            vTaskDelay(pdMS_TO_TICKS(500));  // Delay to simulate a 30-degree turn
            turning_left_timer = false;
        } 
        else {
            // Calculate error and correction based on sensor readings
            calculate_error();
            calculate_correction();

            // Adjust motor speeds based on correction
            left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
            right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);

            // Handle U-turn if no sensor detects the line
            if (line_sensor_readings.adc_reading[0] < BLACK_BOUNDARY &&
                line_sensor_readings.adc_reading[1] < BLACK_BOUNDARY &&
                line_sensor_readings.adc_reading[2] < BLACK_BOUNDARY &&
                line_sensor_readings.adc_reading[3] < BLACK_BOUNDARY &&
                line_sensor_readings.adc_reading[4] < BLACK_BOUNDARY) 
            {
                // U-turn logic
                set_motor_speed(motor_a_0, MOTOR_BACKWARD, 60);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, 60);
                vTaskDelay(pdMS_TO_TICKS(1000));  // Delay to complete a U-turn
            } 
            else {
                set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
            }
        }

        // Debugging output
        ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);

#ifdef CONFIG_ENABLE_OLED
        // Displaying KP, KI, KD values on OLED
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