#include "chute.h"

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
