/*
 *
 */

#ifndef RFCOMM_H
#define RFCOMM_H

#define SPP_SERVER_NAME "OZONO_SPP_SRV"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/

void spp_init();
void spp_end();

#endif
