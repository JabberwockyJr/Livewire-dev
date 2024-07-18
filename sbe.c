/*
 * sbe.c
 *
 *  Created on: Oct 4, 2022
 *      Author: maibui
 */
#include "sbe.h"

//maibui - 4 Oct 2022
/*void  sbe_task (void  *p_arg)
{
  static int i=0;
//  OS_RATE_HZ os_tick_rate;
//  RTOS_ERR p_err;

  int sbe_run_count = 0;
  main_sbe_struct* sbe_ptr;

    // Use argument.
   (void)&p_arg;
   sbe_ptr = p_arg;
   sbe_ptr->sbe_curr_state = 1;
*/
//   os_tick_rate = OSTimeTickRateHzGet (&p_err);

/*
    while (DEF_TRUE) {
        printf("sbe task %d \r\n",i++);
        OSTimeDly (os_tick_rate/10, OS_OPT_TIME_PERIODIC,&p_err);
        // All tasks should be written as infinite loops.
        sbe_run_count++;
        if (sbe_run_count == 10) break;
    }
*/
   /*
    while(DEF_TRUE)
 //    while(!streaming_ptr->streaming_task_done)
     {
         switch(sbe_ptr->sbe_curr_state)
       {
         case init:
 //            printf("streaming_task is in \"init\" state, count %d\r\n",streaming_run_count);
             OSTimeDly (os_tick_rate/10, OS_OPT_TIME_PERIODIC,&p_err);
             // All tasks should be written as infinite loops.
             sbe_run_count++;
             if (sbe_run_count >= 5)
               {
                 sbe_ptr->sbe_curr_state = idle;
                 sbe_run_count = 0;
                 break;
               }
           break;
         case idle:
  //         printf("streaming_task is in \"idle\" state, with run_count %d\r\n",streaming_run_count);
           OSTimeDly (os_tick_rate/10, OS_OPT_TIME_PERIODIC,&p_err);
           // All tasks should be written as infinite loops.
           sbe_run_count++;
           if (sbe_run_count >= 5)
             {
               sbe_ptr->sbe_curr_state = acq;
               sbe_run_count = 0;
               break;
             }
          break;
         case acq:
 //          printf("streaming_task is in \"streaming\" state, count %d\r\n",streaming_run_count);
           OSTimeDly (os_tick_rate/10, OS_OPT_TIME_PERIODIC,&p_err);
           // All tasks should be written as infinite loops.
           sbe_run_count++;
           if (sbe_run_count >= 5)
             {
               sbe_ptr->sbe_curr_state = init;
               sbe_run_count = 0;
               break;
             }
           break;
         default:
           break;
       }
         i++;
     if(i == 30) break;

     }
*/
//}

