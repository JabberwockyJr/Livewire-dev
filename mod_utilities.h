/*
 * mod_utilities.h
 *
 *  Created on: Sep 26, 2022
 *      Author: maibui
 */

#ifndef MOD_UTILITIES_H_
#define MOD_UTILITIES_H_

#include <ff.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>

#define CTD_CBUFSIZE    80
static char ctd_buffer[CTD_CBUFSIZE];
static UINT ctd_bufRead;


int mod_write_sd_f(FIL* fh, char* fn, char* str);

#endif /* MOD_UTILITIES_H_ */
