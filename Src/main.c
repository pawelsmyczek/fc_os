/*TODO:
 * - Basic scheduler: USAGE TO BE CONSIDERED, WHILE INTERRUPTS MATCH OUR NEEDS
 *      * scheduling function IF THERE WILL BE NEED FOR IT, SysTick will be enough for us,
 * - KIND OF FIXED WITH DMA think of ways of better mpu initialisation in terms of scale differences,
 * - Steering algorithms:
 *      * PID ****  DONE  *****
 *      * LQR TO BE IMPLEMENTED AS SOON AS QUADROTOR FLIES WITH PID AND STABILIZES ITSELF
 * - Input for PWM generation:
 *      * IMU BASED, SINCE WE DO EXAMINATIONS FOR ONLY ALITTUDE STABILIZATION
 * - Tests:
 *      * GOOD QUESTION, since we have external flash memory, it can be used to save some test data (as blackbox)
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
    uint16_t pwm_out_high = 1080;
    uint16_t pwm_out_low = 1000;
    char serial_out[] = "";
    int serial_out_len;
    char serial_in_buffer = NULL;
    uint16_t pwm_out_max = 1500;
    uint16_t pwm_out_min = 1018;
    uint16_t pwm_lf = 0, pwm_rf = 0, pwm_lb = 0, pwm_rb = 0;
    uint8_t test_procedure_enabled = 0;
    uint16_t test_procedure_time = 200;
    uint16_t height_increase_time = 100;
    uint16_t ellapsed_time = 0;


    bool success = false;
    static config_t sample_config_file;
    static uint8_t sample_config_buffer[sizeof(config_t)];

    sample_config_file.magic_BE = 0xBE;
    sample_config_file.magic_AC = 0xAC;
    sample_config_file.magic_D3 = 0xD3;
    for (int i = 0; i < 2048; i++)
    {
        sample_config_file.big_array[i] = 2;
        sample_config_file.big_array2[i] = 3;
    }
    //crc calculation
    uint8_t crc = 0;

    for (uint8_t* p = (uint8_t*)&sample_config_file; p < (uint8_t*)&sample_config_file + sizeof(sample_config_file); p++)
    {
        crc ^= *p;
    }

    sample_config_file.crc = crc;


    delay_ms(10000);
    /*
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
    CDC_Send_DATA("- READ FROM FLASH MEMORY?     'r'\n\r", 36);
    CDC_Send_DATA("- TEST GYRO?                  'g'\n\r", 36);
    CDC_Send_DATA("- TEST ACCEL?                 'a'\n\r", 36);
    CDC_Send_DATA("- TEST BARO?                  'b'\n\r", 36);
    while(serial_in_buffer != 't' && serial_in_buffer != 'r' && serial_in_buffer != 'g' && serial_in_buffer != 'a' && serial_in_buffer != 'b')
        CDC_Receive_DATA(&serial_in_buffer, 1);

    delay_ms(1000);
    if(serial_in_buffer == 'r'){
        INFO_LED_ON;
        //sample configuration data
        CDC_Send_DATA("READING FROM EXTERNAL MEMORY PROCEDURE\n\r", 41);

        delay_ms(1000);
        CDC_Send_DATA("WRITING TO EXTERNAL MEMORY\n\r", 29);
        NVIC_DisableIRQ(EXTI4_IRQn);
        write_config((uint8_t*)&sample_config_file, sizeof(config_t));
        read_config(sample_config_buffer, sizeof(config_t), 0x00);
        NVIC_EnableIRQ(EXTI4_IRQn);
        //validation check
        config_t* config_ptr = (config_t*)sample_config_buffer;

        crc = 0;
        for (uint8_t* p = (uint8_t*)config_ptr; p < (uint8_t*)config_ptr + sizeof(sample_config_file); p++)
        {
            crc ^= *p;
        }

        if (config_ptr->magic_AC == sample_config_file.magic_AC && config_ptr->magic_BE == sample_config_file.magic_BE
        && config_ptr->magic_D3 == sample_config_file.magic_D3 && crc == 0)
        {
            INFO_LED_ON;
            success = true;
        }
        else
        {
            INFO_LED_OFF;
            success = false;
        }



    }
    us = micros();
    while(1)
    {
        INFO_LED_ON;
        compute_angles();
        positions_estimate();
        BMP180_GetReadings(&temperature_read, &pressure_read, BMP180_STANDARD);
        /*
         *
         * TEST PROCEDURE START
         * STARTED ONCE ONLY
         *
         * */
        if(test_procedure_enabled == 1){

            /*
             * INCREASE MOTORS VALUES (FUTURE TAKE-OFF ACTION)
             * */

            while(pwm_out++ < pwm_out_high ) {
                compute_angles();
                positions_estimate();
                pwm_rf = pwm_out; pwm_rb = pwm_out; pwm_lb = pwm_out; pwm_lf = pwm_out;
                for(uint8_t motor = 0; motor < 4; motor++)
                    write_motor(motor, pwm_out);
                // sprintf(serial_out, "PWM_RF - %u, PWM_RB - %u, PWM_LB - %u, PWM_LF - %u\n\r", pwm_rf, pwm_rb, pwm_lb, pwm_lf);
                sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", angle[ROLL], angle[PITCH], angle[YAW]);
                CDC_Send_DATA(serial_out, USB_BUFFER_LEN);
                delay_ms(1);
            }

            while(ellapsed_time++ < test_procedure_time){
                compute_angles();
                positions_estimate();
                if(ellapsed_time < height_increase_time) (&pid_z_velocity)->set_point = 20.0f;
                if(ellapsed_time >= height_increase_time) (&pid_z_velocity)->set_point = 0.0f;
                for(uint8_t i = 0; i < 3; i++){
                    (&pid_angle[i])->input = angle[i];
                    pid_compute(&pid_angle[i]);
                }
                (&pid_z_velocity)->input = velocity[YAW];
                pid_compute(&pid_z_velocity);

                pwm_rf = PIDMIX( -1.0f,  1.0f,  1.0f, (pwm_out + (&pid_z_velocity)->output) ); // + (&pid_z_velocity)->output;//pwm_out + (uint16_t)(&pid_angle[ROLL])->output - (uint16_t)(&pid_angle[PITCH])->output;
                pwm_rb = PIDMIX(  1.0f,  1.0f, -1.0f, (pwm_out + (&pid_z_velocity)->output) ); // + (&pid_z_velocity)->output;//pwm_out + (uint16_t)(&pid_angle[ROLL])->output + (uint16_t)(&pid_angle[PITCH])->output;
                pwm_lb = PIDMIX(  1.0f, -1.0f, -1.0f, (pwm_out + (&pid_z_velocity)->output) ); // + (&pid_z_velocity)->output;//pwm_out - (uint16_t)(&pid_angle[ROLL])->output + (uint16_t)(&pid_angle[PITCH])->output;
                pwm_lf = PIDMIX( -1.0f, -1.0f,  1.0f, (pwm_out + (&pid_z_velocity)->output) ); // + (&pid_z_velocity)->output;//pwm_out - (uint16_t)(&pid_angle[ROLL])->output - (uint16_t)(&pid_angle[PITCH])->output;
                if(pwm_rf < pwm_out_min) pwm_rf = pwm_out_min; else if(pwm_rf > pwm_out_max) pwm_rf = pwm_out_max;
                if(pwm_rb < pwm_out_min) pwm_rb = pwm_out_min; else if(pwm_rb > pwm_out_max) pwm_rb = pwm_out_max;
                if(pwm_lb < pwm_out_min) pwm_lb = pwm_out_min; else if(pwm_lb > pwm_out_max) pwm_lb = pwm_out_max;
                if(pwm_lf < pwm_out_min) pwm_lf = pwm_out_min; else if(pwm_lf > pwm_out_max) pwm_lf = pwm_out_max;
                write_motor(MOTOR_1, pwm_rf);
                write_motor(MOTOR_2, pwm_rb);
                write_motor(MOTOR_3, pwm_lb);
                write_motor(MOTOR_4, pwm_lf);
                // sprintf(serial_out, "PWM_RF - %u, PWM_RB - %u, PWM_LB - %u, PWM_LF - %u, Z_VELOCITY - %.1f\n\r", pwm_rf, pwm_rb, pwm_lb, pwm_lf, velocity[YAW]);
                // sprintf(serial_out, "%u, %u, %u, %u, %.1f\n\r", pwm_rf, pwm_rb, pwm_lb, pwm_lf, (&pid_z_velocity)->output);
                sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", angle[ROLL], angle[PITCH], angle[YAW]);
                CDC_Send_DATA(serial_out, USB_BUFFER_LEN);

                delay_ms(100);

            }

            /*
             * DECREASE MOTORS VALUES
             * */


            while(pwm_out-- > pwm_out_low ) {
                compute_angles();
                positions_estimate();
                pwm_rf = pwm_out; pwm_rb = pwm_out; pwm_lb = pwm_out; pwm_lf = pwm_out;
                for (uint8_t motor = 0; motor < 4; motor++)
                    write_motor(motor, pwm_out);
                // sprintf(serial_out, "PWM_RF - %u, PWM_RB - %u, PWM_LB - %u, PWM_LF - %u\n\r", pwm_rf, pwm_rb, pwm_lb, pwm_lf);
                sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", angle[ROLL], angle[PITCH], angle[YAW]);
                CDC_Send_DATA(serial_out, USB_BUFFER_LEN);
                delay_ms(100);
            }

            /*
             * TEST END
             * */

            test_procedure_enabled = 0;
        }

        /*
         *
         * TEST PROCEDURE END
         *
         * */

//        sprintf(serial_out, "%.3f, %.3f, %.3f\n\r", velocity[ROLL], velocity[PITCH], velocity[YAW]);
//        CDC_Send_DATA(serial_out);
//        sprintf(serial_out, "ACCEL[YAW] - %u\n\r", sizeof(float));
        if(serial_in_buffer == 'a'){
            serial_out_len = sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", accelSum[ROLL], accelSum[PITCH], accelSum[YAW]);
            CDC_Send_DATA(serial_out, serial_out_len);
            delay_ms(10);
        }
        if(serial_in_buffer == 'g'){
            serial_out_len = sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", gyroSum[ROLL], gyroSum[PITCH], angle[YAW]);
            CDC_Send_DATA(serial_out, serial_out_len);
            delay_ms(10);
        }
        if(serial_in_buffer == 'b'){
            serial_out_len = sprintf(serial_out, "t: %.1f, p: %.1f\n\r", (float)(temperature_read)/100, (float)(pressure_read)/100);
            CDC_Send_DATA(serial_out, serial_out_len);
            delay_ms(100);
        }
        if(serial_in_buffer == 'r'){
            if(success){
                serial_out_len = sprintf(serial_out, "succesfully written and read %d.%dKB data from external memory\n\r", sizeof(sample_config_file) / 1000, sizeof(sample_config_file) % 1000);
                CDC_Send_DATA(serial_out, serial_out_len);
            }
            else{
                CDC_Send_DATA("failed to read from external memory\n\r", 38);
            }
            delay_ms(1000);
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
