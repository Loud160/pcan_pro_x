#pragma once

#include <stdint.h>
#include "plin.h"

enum 
{
  LIN_BUS_1,
  LIN_BUS_2,
  LIN_BUS_TOTAL
};

void plin_lin_poll( void );
int plin_lin_init( void );

int plin_lin_remap_table( int bus, struct plin_usb_rsp_remap *ptable );
int plin_lin_get_tentry( int bus, struct plin_usb_frm_entry *p );
int plin_lin_set_tentry( int bus, struct plin_usb_frm_entry *p );
int plin_lin_set_filter( int bus, uint8_t *pmask );
int plin_lin_get_filter( int bus, uint8_t *pmask );
int plin_lin_update_entry( int bus, struct plin_usb_update_data *pdata );
int plin_lin_init_ex( int bus, uint32_t bitrate, uint8_t is_master );
void plin_lin_set_mode( int bus, uint8_t is_master );
int plin_lin_write_byte( int bus, uint8_t d );
int plin_lin_write( int bus, struct plin_msg *p_msg );
int plin_lin_read( int bus, struct plin_msg *p_msg );