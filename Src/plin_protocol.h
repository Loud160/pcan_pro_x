#pragma once
#include <stdint.h>

void plin_protocol_init( void );
void plin_protocol_process_cmd( uint8_t *ptr, uint16_t size );
void plin_protocol_process_data( uint8_t *ptr, uint16_t size, uint8_t ep );
void plin_protocol_poll( void );
