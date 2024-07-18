/*
 * mod_utilities.c
 *
 *  Created on: Sep 26, 2022
 *      Author: maibui
 */

#include "mod_utilities.h"


int mod_write_sd_f(FIL* fh, char* fn, char* str)//pass in file name and string to write
{
    int retval = 0;
    unsigned int nwritten;

    printf("inside mod_write_sd_f()\n");

    int num_write = strlen(str);

      if (fh->fptr == 0)
        {
            retval = f_write(&fh, str,num_write,&nwritten);
            if (retval == FR_OK)
            printf("writing data into the file...\n");
            printf("%s\n",str);

            // re read data
            f_rewind(&fh);
            retval = f_read(&fh, ctd_buffer, 10, &ctd_bufRead);
            if (retval == FR_OK)
            {
              PrintBuf(ctd_buffer, ctd_bufRead);
            } else
            {
              printf("Read Failure: %d\n", retval);
            }
      }
      else
      {
        printf("Failed to open %s, error %u\n", fn, retval);
      }

    return retval;
}
