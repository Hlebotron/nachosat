#include "chute.h"

extern QueueHandle_t chute_drq;

float pid_p( float sp, float pv )
{
    return ( sp - pv );
}

float pid_i( float sp, float pv )
{
    static bool init = false;
    static float integral;

    if( !init )
    {
	integral = 0;
	init = true;
    }

    integral += ( sp - pv );
	
    return integral;
}

float pid_d( float sp, float pv )
{
    static float prev_err;
    static TickType_t prev_time;
    static TickType_t current_time = xTaskGetTickCount();
    static float current_err = sp - pv;

    static bool init = false;
    if( !init )
    {
	prev_time = current_time;
	prev_err = current_err;
	init = true;
    }


    float res = ( current_err - prev_err ) / ( current_time - prev_time );
  
    prev_err = current_err;
    prev_time = current_time;
    init = true;
    
    return res;
}

float pid( float sp, float pv )
{
    return ( PID_COEFF_P * pid_p( sp, pv ) + PID_COEFF_I * pid_i( sp, pv ) + PID_COEFF_D * pid_d( sp, pv ) );
}

void ParachuteTask( void* params )
{
    //Deploy parachute
    /* ulTaskNotifyTake(pdTRUE, portMAX_DELAY); */
    /* float integral = 0; */
    /* float pid; */
    /* struct Pos prev_pos, curr_pos; */
    /* struct Pos set_point = GND_POS; */
    /* /\* float err = ; *\/ */
    /* TickType_t prev_time = xTaskGetTickCount(); */
    /* TickType_t curr_time = xTaskGetTickCount(); */
  
    for( ;; )
    {
	/* if (prev_time == curr_time) */
	/*     continue; */
      
	/* int err = read_gps(&curr_pos); */
	/* integral += err; */
	/* pid = err + (COEFF_I * integral) + (COEFF_D * err * 1000 / delta_t); */
	//vTaskDelayUntil(&prev_wake, 20 ms);
    }
}
