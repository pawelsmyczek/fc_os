/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/*TODO:
 * - Basic scheduler:
 *      * typedef struct{
 *          should contain at least:
 *              - task_function
 *              - time_beginning
 *              - time_ellapsed
 *              - (maybe) priority
 *      } task_t;
 *      * typedef struct{
 *              - queue related members
 *      }task_queue;
 *      * tasks_queue inserter,
 *      *task_queue popper,
 *      * scheduling function (is there a good way to do that?),
 * - think of ways of better mpu initialisation in terms of scale differences,
 * - Steering algorithms:
 *      * gyro based
 *      * accell based
 *      * to be consulted
 * - Input for PWM generation:
 *      * maybe structure based ()
 * - Tests
 * */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
//Library config for this project!!!!!!!!!!!
#include "stm32f4xx_conf.h"


/* Private typedef -----------------------------------------------------------*/
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

    setup();

    uint16_t pwm_out = 1000;
    uint16_t pwm_out_high = 1080;
    uint16_t pwm_out_low = 1000;

    uint16_t pwm_out_max = 1500;
    uint16_t pwm_out_min = 1018;
    uint16_t pwm_lf = 0, pwm_rf = 0, pwm_lb = 0, pwm_rb = 0;
    uint8_t test_procedure_enabled = 0;
    uint16_t test_procedure_time = 200;
    uint16_t height_increase_time = 100;
    uint16_t ellapsed_time = 0;
    /*
     *
     * testing throttle values
     * 1010 - minimum value
     * 2000 - maximum value
     * */

    delay_ms(1500);
//    while(pwm_out++ < 1200 ) {
//        for(uint8_t motor = 0; motor < 4; motor++)
//            write_motor(motor, pwm_out);
//        delay_ms(1);
//    }
//    delay_ms(100);
//    while(pwm_out-- > 1000 ) {
//    for(uint8_t motor = 0; motor < 4; motor++)
//        write_motor(motor, pwm_out);
//        delay_ms(1);
//    }

    ms = millis();
    while(1)
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_14);
        setGyroAccelSums();
        positions_estimate();
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
                setGyroAccelSums();
                positions_estimate();
                pwm_rf = pwm_out; pwm_rb = pwm_out; pwm_lb = pwm_out; pwm_lf = pwm_out;
                for(uint8_t motor = 0; motor < 4; motor++)
                    write_motor(motor, pwm_out);
                // sprintf(serial_out, "PWM_RF - %u, PWM_RB - %u, PWM_LB - %u, PWM_LF - %u\n\r", pwm_rf, pwm_rb, pwm_lb, pwm_lf);
                sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", angle[ROLL], angle[PITCH], angle[YAW]);
                VCP_send_str(serial_out);
                delay_ms(1000);
            }

            while(ellapsed_time++ < test_procedure_time){
                setGyroAccelSums();
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
                VCP_send_str(serial_out);

                delay_ms(100);

            }

            /*
             * DECREASE MOTORS VALUES
             * */


            while(pwm_out-- > pwm_out_low ) {
                setGyroAccelSums();
                positions_estimate();
                pwm_rf = pwm_out; pwm_rb = pwm_out; pwm_lb = pwm_out; pwm_lf = pwm_out;
                for (uint8_t motor = 0; motor < 4; motor++)
                    write_motor(motor, pwm_out);
                // sprintf(serial_out, "PWM_RF - %u, PWM_RB - %u, PWM_LB - %u, PWM_LF - %u\n\r", pwm_rf, pwm_rb, pwm_lb, pwm_lf);
                sprintf(serial_out, "%.1f, %.1f, %.1f\n\r", angle[ROLL], angle[PITCH], angle[YAW]);
                VCP_send_str(serial_out);
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
//        VCP_send_str(serial_out);
        sprintf(serial_out, "GYRO[YAW] - %.1f\n\r", angle[YAW]);
        VCP_send_str(serial_out);

        GPIO_ResetBits(GPIOC, GPIO_Pin_14);

        delay_ms(1);
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

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
