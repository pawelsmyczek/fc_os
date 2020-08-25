/*TODO:
 * - Basic scheduler: USAGE TO BE CONSIDERED, WHILE INTERRUPTS MATCH OUR NEEDS
 *      * scheduling function IF THERE WILL BE NEED FOR IT, SysTick will be enough for mpu_timestamp,
 * - KIND OF FIXED WITH DMA think of ways of better mpu initialization in terms of scale differences,
 * - Steering algorithms:
 *      * LQR TO BE IMPLEMENTED AS SOON AS QUADROTOR FLIES WITH PID AND STABILIZES ITSELF
 * - Tests
 * */


#include "main.h"

typedef struct
{
    uint8_t magic_BE;
    uint8_t big_array[2048];
    uint8_t magic_AC;
    uint8_t big_array2[2048];
    uint8_t magic_D3;
    uint8_t crc;
} config_t;


typedef struct
{
    float height_data[1024];
    float angle_roll[ 1024];
    float angle_pitch[1024];
    float angle_yaw[  1024];
} flight_data_t;



#define USB_BUFFER_LEN  255

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/




/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */


/* Private functions ---------------------------------------------------------*/

int main(void)
{

    system_init();

    uint16_t pwm_out = 1000;
    uint16_t pwm_out_high = 1275;
    uint16_t pwm_out_low = 1000;
    unsigned char serial_out[70];
    int serial_out_len;
    int serial_in_len;
    int bmp_calibration_time = 10000;
    char serial_in_buffer = NULL;
    char serial_in_buffer_mul[2] = "";
    uint16_t pwm_out_max = 1500;
    uint16_t pwm_out_min = 1018;
    uint16_t pwm_lf = 0, pwm_rf = 0, pwm_lb = 0, pwm_rb = 0;
    uint32_t test_procedure_time = 15000;
    uint16_t height_increase_time = 100;
    uint32_t ellapsed_time = 0;



    bool success_flash_read = false;
//    static config_t sample_config_file;
//    static uint8_t sample_config_buffer[sizeof(config_t)];
    static flight_data_t flight_data_file;
    static uint8_t flight_data_buffer[sizeof(flight_data_file)];

//    sample_config_file.magic_BE = 0xBE;
//    sample_config_file.magic_AC = 0xAC;
//    sample_config_file.magic_D3 = 0xD3;
//    for (int i = 0; i < 2048; i++)
//    {
//        sample_config_file.big_array[i] = 2;
//        sample_config_file.big_array2[i] = 3;
//    }
    //crc calculation
//    uint8_t crc = 0;
//
//    for (uint8_t* p = (uint8_t*)&sample_config_file; p < (uint8_t*)&sample_config_file + sizeof(sample_config_file); p++)
//        crc ^= *p;
//
//    sample_config_file.crc = crc;


    delay_ms(10000);


    /**
     *
     * tested throttle values
     * 1010 - minimum value
     * 2000 - maximum value
     * */

    CDC_Send_DATA("FLIGHT CONTROL PROGRAM BEGIN - type 'c' to continue\n\r", 54);
    while(serial_in_buffer != 'c') CDC_Receive_DATA(&serial_in_buffer, 1);

    //delay_ms(1000);
    CDC_Send_DATA("WHAT WOULD YOU LIKE TO DO? TYPE ONE OF THESE:\n\r", 48);
    CDC_Send_DATA("- BEGIN TEST FLIGHT?          't'\n\r", 36);
    CDC_Send_DATA("  * REMEMBER TO PLACE QUAD ON STABLE POSITION\n\r", 48);
    CDC_Send_DATA("  * TEST FLIGHT BEGINS IN 20 SECONDS\n\r", 39);
    CDC_Send_DATA("- READ FROM FLASH MEMORY?     'r'\n\r", 36);
    CDC_Send_DATA("- TEST GYRO?                  'g'\n\r", 36);
    CDC_Send_DATA("- TEST ACCEL?                 'a'\n\r", 36);
    CDC_Send_DATA("- TEST BARO?                  'b'\n\r", 36);
    while(serial_in_buffer != 't' && serial_in_buffer != 'r' && serial_in_buffer != 'g' && serial_in_buffer != 'a' && serial_in_buffer != 'b')
        CDC_Receive_DATA(&serial_in_buffer, 1);

    delay_ms(1000);
    if(serial_in_buffer == 'r'){
        CDC_Send_DATA("------------READING FROM BLACK BOX-------------\n\r", 50);
        CDC_Send_DATA("THESE WERE WRITTEN TO MEMORY DURING LAST FLIGHT\n\r", 50);
        CDC_Send_DATA("- HEIGHT?                        'ht'\n\r", 39);
        CDC_Send_DATA("- ANGLE? (ROLL PITCH YAW)        'ag'\n\r", 39);
        CDC_Send_DATA("- ACCELERATION? (ROLL PITCH YAW) 'ac'\n\r", 39);
        CDC_Send_DATA("- ALL?                           'al'\n\r", 39);
        CDC_Send_DATA("-----------------------------------------------\n\r", 50);
//        while(strcmp(serial_in_buffer_mul, "ht") != 0  && strcmp(serial_in_buffer_mul, "ag") != 0 &&
//                                strcmp(serial_in_buffer_mul,"ac") != 0 && strcmp(serial_in_buffer_mul, "al") != 0)
//            CDC_Receive_DATA(&serial_in_buffer_mul, 2);
        INFO_LED_ON;
        //sample configuration data
        serial_out_len = sprintf(serial_out, "READING FROM EXTERNAL MEMORY PROCEDURE, option chosen: %s\n\r", serial_in_buffer_mul);
        CDC_Send_DATA(serial_out, serial_out_len);

        delay_ms(1000);
        NVIC_DisableIRQ(EXTI4_IRQn);
        read_config(flight_data_buffer, sizeof(flight_data_t), 0x00);
        NVIC_EnableIRQ(EXTI4_IRQn);
        flight_data_t* config_ptr = (flight_data_t*)flight_data_buffer;
        for(int i = 0; i < 1024; i++){
            serial_out_len = sprintf(serial_out, "%.2f, %.2f, %.2f, %.2f\n\r",config_ptr->angle_roll[i], config_ptr->angle_pitch[i],config_ptr->angle_yaw[i], config_ptr->height_data[i]);
            // sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", angle[ROLL], angle[PITCH], angle[YAW]);
            CDC_Send_DATA(serial_out, serial_out_len);
            delay_ms(10);
        }
        //validation check
//        config_t* config_ptr = (config_t*)sample_config_buffer;

//        crc = 0;
//        for (uint8_t* p = (uint8_t*)config_ptr; p < (uint8_t*)config_ptr + sizeof(sample_config_file); p++)
//            crc ^= *p;
//
//        if (config_ptr->magic_AC == sample_config_file.magic_AC && config_ptr->magic_BE == sample_config_file.magic_BE
//        && config_ptr->magic_D3 == sample_config_file.magic_D3 && crc == 0)
//        {
//            INFO_LED_ON;
//            success_flash_read = true;
//        }
//        else
//        {
//            INFO_LED_OFF;
//            success_flash_read = false;
//        }
    }

    /**
     *
     * TEST PROCEDURE START
     * STARTED ONCE ONLY
     *
     * */
    if(serial_in_buffer == 't'){

        CDC_Send_DATA("FLIGHT TESTING PROCEDURE BEGINS IN 20 SEC\n\r", 44);
        delay_ms(10000);

        /**
         *
         * Calibration of barometer ,,pressure_average_slow'' takes about 5 seconds
         * After the calibration, the ,,ground altitude'' is set
         * */
        bmp_calibration_time+=millis();
        while (millis() < bmp_calibration_time){
            bmp180_update();
            bmp_data.ground_altitude = bmp_data.low_pass_filtered;
        }
        (&pid_altitude)->set_point = 0.4f;
        /**
         * INCREASE MOTORS VALUES (FUTURE TAKE-OFF ACTION)
         * */

        while(pwm_out++ < pwm_out_high ) {
            bmp180_update();
            compute_angles();
            positions_estimate();
            pwm_rf = pwm_out; pwm_rb = pwm_out; pwm_lb = pwm_out; pwm_lf = pwm_out;
            for(uint8_t motor = 0; motor < 4; motor++)
                write_motor(motor, pwm_out);
            serial_out_len = sprintf(serial_out, "PWM_RF - %u, PWM_RB - %u, PWM_LB - %u, PWM_LF - %u\n\r", pwm_rf, pwm_rb, pwm_lb, pwm_lf);
            // serial_out_len = sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", angle[ROLL], angle[PITCH], angle[YAW]);
            CDC_Send_DATA(serial_out, serial_out_len);
            delay_ms(10);
        }
        uint16_t incr_data = 0;
        ellapsed_time = millis() + test_procedure_time;
        while(millis() < ellapsed_time){
            bmp180_update();
            compute_angles();
            positions_estimate();
            /**
             * start height increase until the set point is reached
             * */
            if(bmp_data.delta_altitude < (&pid_altitude)->set_point - 0.2f){
                if(pwm_out < pwm_out_high + 30)
                    pwm_out++;
            } else{
                pwm_out = pwm_out_high;
            }
            /**
             * end height increase until the set point is reached
             * */
            for(uint8_t i = 0; i < 3; i++){
                (&pid_angle[i])->input = i == YAW? angle[i] : angle_from_rot[i];
                pid_compute(&pid_angle[i]);
                (&pid_angle[i])->set_point =  0.0f;
            }

            (&pid_altitude)->input =  bmp_data.delta_altitude;
            pid_compute(&pid_altitude);
            // (&pid_z_velocity)->input =  velocity[YAW];

            pwm_rf = PIDMIX( -1.0f,  1.0f,  1.0f, pwm_out + (&pid_altitude)->output); // + (&pid_z_velocity)->output;//pwm_out + (uint16_t)(&pid_angle[ROLL])->output - (uint16_t)(&pid_angle[PITCH])->output;
            pwm_rb = PIDMIX(  1.0f,  1.0f, -1.0f, pwm_out + (&pid_altitude)->output); // + (&pid_z_velocity)->output;//pwm_out + (uint16_t)(&pid_angle[ROLL])->output + (uint16_t)(&pid_angle[PITCH])->output;
            pwm_lb = PIDMIX(  1.0f, -1.0f, -1.0f, pwm_out + (&pid_altitude)->output); // + (&pid_z_velocity)->output;//pwm_out - (uint16_t)(&pid_angle[ROLL])->output + (uint16_t)(&pid_angle[PITCH])->output;
            pwm_lf = PIDMIX( -1.0f, -1.0f,  1.0f, pwm_out + (&pid_altitude)->output); // + (&pid_z_velocity)->output;//pwm_out - (uint16_t)(&pid_angle[ROLL])->output - (uint16_t)(&pid_angle[PITCH])->output;
            if(pwm_rf < pwm_out_min) pwm_rf = pwm_out_min; else if(pwm_rf > pwm_out_max) pwm_rf = pwm_out_max;
            if(pwm_rb < pwm_out_min) pwm_rb = pwm_out_min; else if(pwm_rb > pwm_out_max) pwm_rb = pwm_out_max;
            if(pwm_lb < pwm_out_min) pwm_lb = pwm_out_min; else if(pwm_lb > pwm_out_max) pwm_lb = pwm_out_max;
            if(pwm_lf < pwm_out_min) pwm_lf = pwm_out_min; else if(pwm_lf > pwm_out_max) pwm_lf = pwm_out_max;
            write_motor(MOTOR_1, pwm_rf);
            write_motor(MOTOR_2, pwm_rb);
            write_motor(MOTOR_3, pwm_lb);
            write_motor(MOTOR_4, pwm_lf);
            if(incr_data < 1024){
                flight_data_file.height_data[incr_data] = bmp_data.delta_altitude;
                flight_data_file.angle_roll[incr_data]  = angle_from_rot[ROLL];
                flight_data_file.angle_pitch[incr_data] = angle_from_rot[PITCH];
                flight_data_file.angle_yaw[incr_data]   = angle[YAW];
            }
            // serial_out_len = sprintf(serial_out, "PWM_RF - %u, PWM_RB - %u, PWM_LB - %u, PWM_LF - %u, Z_VELOCITY - %.1f\n\r", pwm_rf, pwm_rb, pwm_lb, pwm_lf, velocity[YAW]);
            // serial_out_len = sprintf(serial_out, "%u, %u, %u, %u, %.1f\n\r", pwm_rf, pwm_rb, pwm_lb, pwm_lf, (&pid_z_velocity)->output);
            // serial_out_len = sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", gyro_sum[ROLL], gyro_sum[PITCH], gyro_sum[YAW]);
            // serial_out_len = sprintf(serial_out, "%u, %u, %u, %u, %.1f, %.1f, %.1f\n\r",pwm_rf, pwm_rb, pwm_lb, pwm_lf, angle[ROLL], angle[PITCH], angle[YAW]);
            serial_out_len = sprintf(serial_out, "%.2f, %.1f, %.1f\n\r",(&pid_altitude)->output,
                    (&pid_angle[ROLL])->output, (&pid_angle[PITCH])->output);
//            serial_out_len = sprintf(serial_out, "%.2f, %.2f\n\r",(&pid_altitude)->output,
//                                     bmp_data.delta_altitude);
            CDC_Send_DATA(serial_out, serial_out_len);
            incr_data++;
            delay_ms(10);
        }

        /**
         * DECREASE MOTORS VALUES
         * */


        while(pwm_out-- > pwm_out_low ) {
            bmp180_update();
            compute_angles();
            positions_estimate();
            pwm_rf = pwm_out; pwm_rb = pwm_out; pwm_lb = pwm_out; pwm_lf = pwm_out;
            for (uint8_t motor = 0; motor < 4; motor++)
                write_motor(motor, pwm_out);
            serial_out_len = sprintf(serial_out, "%u, %u, %u, %u\n\r", pwm_rf, pwm_rb, pwm_lb, pwm_lf);
//            serial_out_len = sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", angle[ROLL], angle[PITCH], angle[YAW]);
            CDC_Send_DATA(serial_out, serial_out_len);
            delay_ms(10);
        }
        /*
         * TEST END
         * */
//            serial_out_len = sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", angle[ROLL], angle[PITCH], angle[YAW]);
        CDC_Send_DATA("START WRITING FLIGHT DATA TO EEPROM\n\r", 38);

        NVIC_DisableIRQ(EXTI4_IRQn);
        write_config((uint8_t*)&flight_data_file, sizeof(flight_data_t));
        NVIC_EnableIRQ(EXTI4_IRQn);

        CDC_Send_DATA("WRITING FLIGHT DATA TO EEPROM END\n\r", 36);
    }

    /*
     *
     * TEST PROCEDURE END
     *
     * */

    bmp_calibration_time+=millis();
    while (millis() < bmp_calibration_time){
        bmp180_update();
        bmp_data.ground_altitude = bmp_data.low_pass_filtered;
    }


    while(1)
    {
        INFO_LED_ON;
        bmp180_update();
        // BMP180_GetReadings(&bmp_data.temperature_read, &bmp_data.pressure_read, BMP180_STANDARD);
        compute_angles();
        positions_estimate();

//        sprintf(serial_out, "%.3f, %.3f, %.3f\n\r", velocity[ROLL], velocity[PITCH], velocity[YAW]);
//        CDC_Send_DATA(serial_out);
//        sprintf(serial_out, "ACCEL[YAW] - %u\n\r", sizeof(float));
        if(serial_in_buffer == 'a'){
            serial_out_len = sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", position[ROLL], position[PITCH], position[YAW]);
            CDC_Send_DATA(serial_out, serial_out_len);
            delay_ms(10);
        }
        if(serial_in_buffer == 'g'){
            serial_out_len = sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", angle_from_rot[ROLL], angle_from_rot[PITCH], angle_from_rot[YAW]);
            CDC_Send_DATA(serial_out, serial_out_len);
            delay_ms(10);
        }
        if(serial_in_buffer == 'b'){
            serial_out_len = sprintf(serial_out, "%.2f\n\r", bmp_data.delta_altitude);
            CDC_Send_DATA(serial_out, serial_out_len);
            delay_ms(10);
        }
        INFO_LED_OFF;
	}


}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
