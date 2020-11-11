//
// Created by pablito on 21.04.2020.
//

#include "pid.h"

//*********************************************************************************
// Macros and Globals
//*********************************************************************************


void pid_init(PIDControl *pid, float kp, float ki, float kd,
              float sample_time_seconds, float min_output, float max_output,
              PIDMode mode, PIDDirection controller_direction){

    pid->controller_direction = controller_direction;
    pid->mode = mode;
    pid->i_term = 0.0f;
    pid->input = 0.0f;
    pid->previous_input = 0.0f;
    pid->output = 0.0f;
    pid->set_point = 0.0f;

    if(sample_time_seconds > 0.0f){
        pid->sample_time = sample_time_seconds;
    } else {
        pid->sample_time = 1.0f;
    }


    pid_output_limits_set(pid, min_output, max_output);
    pid_tuning_set(pid, kp, ki, kd);

}

bool pid_compute(PIDControl *pid){

    float error, dInput;

    if(pid->mode == MANUAL)
    {
        return false;
    }

    // The classic PID error term
    error = (pid->set_point) - (pid->input);

    // Compute the integral term separately ahead of time
    pid->i_term += (pid->altered_ki) * error;

    // Constrain the integrator to make sure it does not exceed output bounds
    pid->i_term = CONSTRAIN( (pid->i_term), (pid->i_term), (pid->out_max) );

    // Take the "derivative on measurement" instead of "derivative on error"
    dInput = (pid->input) - (pid->previous_input);

    // Run all the terms together to get the overall output
    pid->output = (pid->altered_kp) * error + (pid->i_term) - (pid->altered_kd) * dInput;

    // Bound the output
    pid->output = CONSTRAIN( (pid->output), (pid->out_min), (pid->out_max) );

    // Make the current input the former input
    pid->previous_input = pid->input;

    return true;

}


void pid_output_limits_set(PIDControl *pid, float min, float max)
{
    // Check if the parameters are valid
    if(min >= max)
    {
        return;
    }

    // Save the parameters
    pid->out_min = min;
    pid->out_max = max;

    // If in automatic, apply the new constraints
    if(pid->mode == AUTOMATIC)
    {
        pid->output = CONSTRAIN(pid->output, min, max);
        pid->i_term  = CONSTRAIN(pid->i_term,  min, max);
    }
}

void pid_tuning_set(PIDControl *pid, float kp, float ki, float kd)
{
    // Check if the parameters are valid
    if(kp < 0.0f || ki < 0.0f || kd < 0.0f)
    {
        return;
    }


    // Alter the parameters for PID
    pid->altered_kp = kp;
    pid->altered_ki = ki * pid->sample_time;
    pid->altered_kd = kd / pid->sample_time;

    // Apply reverse direction to the altered values if necessary
    if(pid->controller_direction == REVERSE)
    {
        pid->altered_kp = -(pid->altered_kp);
        pid->altered_ki = -(pid->altered_ki);
        pid->altered_kd = -(pid->altered_kd);
    }
}



