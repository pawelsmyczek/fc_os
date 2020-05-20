//
// Created by pablito on 21.04.2020.
//

#ifndef FC_SOFT_PID_H
#define FC_SOFT_PID_H


#include <stdbool.h>

#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))
#define PIDMIX(X,Y,Z,T) ((&pid_angle[ROLL])->output * (X) + (&pid_angle[PITCH])->output * (Y) + (&pid_angle[YAW])->output * (Z) + (T))


typedef enum {
    MANUAL,
    AUTOMATIC
} PIDMode;

typedef enum {
    DIRECT,
    REVERSE
} PIDDirection;


typedef struct{
    float input;
    float previous_input;
    float output;


    float altered_kp;
    float altered_ki;
    float altered_kd;

    float i_term;

    float sample_time;

    float out_min;
    float out_max;

    float set_point;

    PIDDirection controller_direction;
    PIDMode mode;
}PIDControl;


/**
 * PID Initialize
 * @brief
 *     Initializes a PIDControl instantiation. This should be called at least once
 *     before any other PID functions are called.
 * @param
 *     pid - The address of a PIDControl instantiation.
 *     kp - Positive P gain constant value.
 *     ki - Positive I gain constant value.
 *     kd - Positive D gain constant value.
 *     sample_time_seconds - Interval in seconds on which pid_compute will be called.
 *     min_output - Constrain PID output to this minimum value.
 *     max_output - Constrain PID output to this maximum value.
 *     mode - Tells how the controller should respond if the user has taken over
 *            manual control or not.
 *            MANUAL:    PID controller is off. User can manually control the
 *                       output.
 *            AUTOMATIC: PID controller is on. PID controller controls the output.
 *     controller_direction - The sense of direction of the controller
 *                           DIRECT:  A positive setpoint gives a positive output.
 *                           REVERSE: A positive setpoint gives a negative output.
 * @retval None
 *
 * */

void pid_init(PIDControl *pid, float kp, float ki, float kd,
              float sample_time_seconds, float min_output, float max_output,
              PIDMode mode, PIDDirection controller_direction);
/**
  * PID Compute
  * @brief
  *      Should be called on a regular interval defined by sample_time_seconds.
  *      Typically, PIDSetpointSet and PIDInputSet should be called immediately
  *      before PIDCompute.
  * @param pid - The address of a PIDControl instantiation.
  * @retval True if in AUTOMATIC. False if in MANUAL.
  *
 **/
bool pid_compute(PIDControl *pid);


/**
  * PID Output Limits Set
  * @brief
  *      Sets the new output limits. The new limits are applied to the PID
  *      immediately.
  * @param
  *      pid - The address of a PIDControl instantiation.
  *      min - Constrain PID output to this minimum value.
  *      max - Constrain PID output to this maximum value.
  * @retval None.
 * */

void pid_output_limits_set(PIDControl *pid, float min, float max);

/**
  * PID Tunings Set
  * @brief
  *      Sets the new gain constant values.
  * @param
  *      pid - The address of a PIDControl instantiation.
  *      kp - Positive P gain constant value.
  *      ki - Positive I gain constant value.
  *      kd - Positive D gain constant value.
  * @retval None.
* */

void pid_tuning_set(PIDControl *pid, float kp, float ki, float kd);

/**
 * PID Setpoint Set
 * @brief
 *      Alters the setpoint the PID controller will try to achieve.
 * @param
 *      pid - The address of a PIDControl instantiation.
 *      setpoint - The desired setpoint the PID controller will try to obtain.
 * @retval
 *      Nothing.
**/
inline void pid_set_point_set(PIDControl *pid, float setpoint) { pid->set_point = setpoint; }

/**
* PID Input Set
* @brief
*      Should be called before calling PIDCompute so the PID controller will
*      have an updated input value to work with.
* @param
*      pid - The address of a PIDControl instantiation.
*      input - The value the controller will work with.
* @retval
*      Nothing.
**/
inline void pid_input_set(PIDControl *pid, float input) { pid->input = input; }

/**
* PID Output Get
* @brief
*      Typically, this function is called after PIDCompute in order to
*      retrieve the output of the controller.
* @param
*      pid - The address of a PIDControl instantiation.
* @retval
*      The output of the specific PID controller.
**/
inline float pid_output_set(PIDControl *pid) { return pid->output; }

/**
* PID Proportional Gain Constant Get
* @brief
*      Returns the proportional gain constant value the particular
*      controller is set to.
* @param
*      pid - The address of a PIDControl instantiation.
* @retval
*      The proportional gain constant.
**/
inline float pid_kp_get(PIDControl *pid) { return pid->altered_kp; }

/**
* PID Integral Gain Constant Get
* @brief
*      Returns the integral gain constant value the particular
*      controller is set to.
* @param
*      pid - The address of a PIDControl instantiation.
* @retval
*      The integral gain constant.
**/
inline float pid_ki_get(PIDControl *pid) { return pid->altered_ki; }

/**
* PID Derivative Gain Constant Get
* @brief
*      Returns the derivative gain constant value the particular
*      controller is set to.
* @param
*      pid - The address of a PIDControl instantiation.
* @retval
*      The derivative gain constant.
**/
inline float pid_kd_get(PIDControl *pid) { return pid->altered_kd; }

/**
* PID Mode Get
* @brief
*      Returns the mode the particular controller is set to.
* @param
*      pid - The address of a PIDControl instantiation.
* @retval
*      MANUAL or AUTOMATIC depending on what the user set the
*      controller to.
**/
inline PIDMode pid_mode_get(PIDControl *pid) { return pid->mode; }

/**
* PID Direction Get
* @brief
*      Returns the direction the particular controller is set to.
* @param
*      pid - The address of a PIDControl instantiation.
* @retval
*      DIRECT or REVERSE depending on what the user set the
*      controller to.
**/
inline PIDDirection pid_direction_get(PIDControl *pid) { return pid->controller_direction; }


#endif //FC_SOFT_PID_H
