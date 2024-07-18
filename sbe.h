/*
 * sbe.h
 *
 *  Created on: Sep 29, 2022
 *      Author: maibui
 */

#ifndef SBE_H_
#define SBE_H_

#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
//#include <os_type.h>

#include "lib_def.h"

//enum sbe_task_state {ident=0, read, checking, saving, streaming};
/*
enum sbe_task_state{ident=0, read, checking, saving, sbe_streaming};

typedef struct main_sbe_struct
{
    bool sbe_task_done;

    enum sbe_task_state sbe_curr_state ;
}main_sbe_struct, *main_sbe_struct_ptr;

void  sbe_task (void  *p_arg); //maibui - 4 Oct 2022
void  sbe_task_create (main_sbe_struct *); //maibui - 4 Oct 2022
*/
#endif /* SBE_H_ */
