#include <assert.h>
#include <string.h>
#include "plin_protocol.h"
#include "pcanpro_usbd.h"
#include "plin.h"
#include "plin_usb.h"
#include "plin_lin.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_led.h"
#include "usb_device.h"

/* cmd response max - 128, data response max - 256 */

#define LIN_DEFAULT_BAUDRATE (19200u)

static struct
{
  uint8_t  lin_drv_loaded;

  struct
  {
    /* config cache */
    uint8_t   mode;
    uint8_t   bus;
    uint32_t  baudrate;
    struct plin_usb_ident_str   ident;
    struct plin_usb_keep_alive  keep_alive;
    uint32_t last_ka_ts;
    int      enable_ka_frame;
  }
  lin[LIN_BUS_TOTAL];
}
lin_device =
{
  .lin[0] = 
  {
    .mode = PLIN_MODE_NONE,
    .baudrate = LIN_DEFAULT_BAUDRATE,
  },
  .lin[1] =
  { 
    .mode = PLIN_MODE_NONE,
    .baudrate = LIN_DEFAULT_BAUDRATE,
  },
};

#define PLIN_DATA_BUFFER_SIZE (256)
static uint8_t  resp_buffer[PLIN_DATA_BUFFER_SIZE];
static uint8_t  msg_buffer[PLIN_DATA_BUFFER_SIZE];
static uint16_t msg_pos = 4;

static struct t_m2h_fsm resp_fsm[1] = 
{
  [0] = {
    .state = 0,
    .ep_addr = PLIN_USB_EP_MSGIN_CH1,
    .pdbuf = resp_buffer,
    .dbsize = PLIN_DATA_BUFFER_SIZE,
  }
};

/* send back data format */
/*
  uint8_t [0]buf_type;
  uint8_t [1]buf_count; // direct message counter 
  uint8_t [2]msg_count; // how many messages in buffer
  uint8_t [3]unknow;
*/
#define PLIN_RXMSG_SIZE   ( 32 )
#define PLIN_ID_MASK      (PLIN_FRM_ID_MAX)
int plin_process_rx_message( int bus,struct plin_msg *pmsg )
{
  uint8_t *ptr = msg_buffer;

  if( ( msg_pos + PLIN_RXMSG_SIZE ) > sizeof( msg_buffer ) )
    return -1;
  
  /* update message counter */
  ++ptr[2]; /* how many message for this buffer */

  ptr += msg_pos;

  ptr[0] = pmsg->type;

  ptr[1] = 0x00; /* unknow */

  ptr[2] = bus;

  ptr[3] = 0x00; /* unknow */

  ptr[4] = pmsg->id;
  ptr[5] = pmsg->len;
  ptr[6] = pmsg->dir;
  ptr[7] = pmsg->cs_type;

  ptr[8] = pmsg->reserved[0]; /* checksum */
  ptr[9] = 0; /* unknow */
  ptr[10] = 0; /* unknow */
  ptr[11] = 0; /* unknow */

  /* flags */
  ptr[12] = pmsg->flags & 0xFF;
  ptr[13] = (pmsg->flags >> 8) & 0xFF;; 
  ptr[14] = 0;
  ptr[15] = 0;

  /* ts 64bit us */
  ptr[16] = pmsg->ts_us & 0xFF;
  ptr[17] = ( pmsg->ts_us >> 8) & 0xFF;
  ptr[18] = ( pmsg->ts_us >> 16) & 0xFF;
  ptr[19] = ( pmsg->ts_us >> 24) & 0xFF;
  ptr[20] = ( pmsg->ts_us >> 32) & 0xFF;
  ptr[21] = ( pmsg->ts_us >> 40) & 0xFF;
  ptr[22] = ( pmsg->ts_us >> 48) & 0xFF;
  ptr[23] = ( pmsg->ts_us >> 56) & 0xFF;

  /* payload */
  memcpy( &ptr[24], pmsg->data, PLIN_DAT_LEN );

  msg_pos += PLIN_RXMSG_SIZE;
  return 0;
}

int plin_buffer_reset( void )
{
  uint8_t *ptr = msg_buffer;
  ptr[0] = 0;
  ++ptr[1]; /* just a global message counter */
  ptr[2] = 0; /* in-message counter */
  ptr[3] = 0;

  msg_pos = 4;
  return 0;
}

void plin_protocol_init( void )
{
  for( int i = 0; i < LIN_BUS_TOTAL; i++ )
  {
    /* last 4 bytes is channel serial number */
    memset( lin_device.lin[i].ident.str, 0xFF, sizeof( lin_device.lin->ident.str ) );
  }

  pcan_led_set_mode( LED_LIN_LED0, LED_MODE_OFF, 0 );
  pcan_led_set_mode( LED_LIN_LED1, LED_MODE_OFF, 0 );

  plin_lin_init();
}

static int plin_packet_size( int id )
{
  switch( id )
  {
    case PLIN_USB_CMD_GET_FRM_ENTRY:
      return 4 + sizeof( struct plin_usb_frm_entry );
    case PLIN_USB_CMD_START_AUTO_BAUD:
      return 4 + sizeof( struct plin_usb_auto_baud );
    case PLIN_USB_CMD_GET_BAUDRATE:
      return 4 + sizeof( struct plin_usb_get_baudrate );
    case PLIN_USB_CMD_GET_ID_FILTER:
      return 4 + sizeof( struct plin_usb_id_filter );
    case PLIN_USB_CMD_GET_MODE:
      return 4 + sizeof( struct plin_usb_get_mode );
    case PLIN_USB_CMD_GET_HW_IDENT:
    case PLIN_USB_CMD_GET_IDENT_STR:
      return 4 + sizeof( struct plin_usb_ident_str );
    case PLIN_USB_CMD_GET_FW_VER:
      return 4 + sizeof( struct plin_usb_fw_ver );
    case PLIN_USB_CMD_GET_STATUS:
      return 4 + sizeof( struct plin_usb_get_status );
    case PLIN_USB_CMD_RSP_REMAP:
      return 4 + sizeof( struct plin_usb_rsp_remap );
    default:
      assert( 0 );
  }
  return 0;
}

void plin_protocol_process_cmd( uint8_t *ptr, uint16_t size )
{
#define PLIN_RESP_BUFFER_SIZE (128)
  static uint8_t cmd_resp_buffer[PLIN_RESP_BUFFER_SIZE];
  struct plin_usb_cmd  *pcmd = (void*)ptr;
  struct plin_usb_cmd  *pcmd_resp = (void*)cmd_resp_buffer;

  assert( sizeof(struct plin_usb_cmd ) <= PLIN_RESP_BUFFER_SIZE );
  assert( size <= PLIN_RESP_BUFFER_SIZE );
  assert( pcmd->bus < LIN_BUS_TOTAL );

  /* copy header */
  memcpy( pcmd_resp, pcmd, 0x04 );

  switch( pcmd->id )
  {
    case PLIN_USB_CMD_INIT_HW:
      lin_device.lin[pcmd->bus].mode = pcmd->init_hw.mode;
      lin_device.lin[pcmd->bus].baudrate = pcmd->init_hw.baudrate;
      plin_lin_init_ex( pcmd->bus, pcmd->init_hw.baudrate, pcmd->init_hw.mode );
      break;
    case PLIN_USB_CMD_RST_HW_CFG:
      lin_device.lin[pcmd->bus].mode = PLIN_MODE_NONE;
      lin_device.lin[pcmd->bus].baudrate = LIN_DEFAULT_BAUDRATE;

      plin_lin_init_ex( pcmd->bus, LIN_DEFAULT_BAUDRATE, PLIN_MODE_NONE );
      break;
    /* global table entry commands */
    case PLIN_USB_CMD_SET_FRM_ENTRY:
      plin_lin_set_tentry( pcmd->bus, &pcmd->frm_entry );
      break;
    case PLIN_USB_CMD_GET_FRM_ENTRY:
      pcmd_resp->frm_entry.id = pcmd->frm_entry.id;
      plin_lin_get_tentry( pcmd->bus, &pcmd_resp->frm_entry );

      pcan_data_trasmit( PLIN_USB_EP_CMDIN, pcmd_resp, plin_packet_size( pcmd->id  ) );
      break;
    case PLIN_USB_CMD_START_AUTO_BAUD:
    {
      pcmd_resp->auto_baud.timeout = pcmd->auto_baud.timeout;
      pcmd_resp->auto_baud.err = pcmd->auto_baud.err;
      pcmd_resp->auto_baud.unused = pcmd->auto_baud.unused;
      /* ack autobaudrate */
      pcan_data_trasmit( PLIN_USB_EP_CMDIN, pcmd_resp, plin_packet_size( pcmd->id  ) );

      /* TODO: response to PLIN_USB_EP_MSGIN_CH1 with result or if timeout reached */
      struct plin_msg msg = { 0 };
      msg.type = PLIN_MSG_AUTOBAUD_TO;
      msg.ts_us = pcan_timestamp_us();
      plin_process_rx_message( pcmd->bus, &msg );
    }
      break;
    case PLIN_USB_CMD_GET_BAUDRATE:
      pcmd_resp->get_baudrate.baudrate = lin_device.lin[pcmd->bus].baudrate;
      pcmd_resp->get_baudrate.unused = 0;
      pcan_data_trasmit( PLIN_USB_EP_CMDIN, pcmd_resp, plin_packet_size( pcmd->id  ) );
      break;
    case PLIN_USB_CMD_SET_ID_FILTER:
      plin_lin_set_filter( pcmd->bus, pcmd->id_filter.id_mask );
      break;
    case PLIN_USB_CMD_GET_ID_FILTER:
      plin_lin_get_filter( pcmd->bus, pcmd_resp->id_filter.id_mask );
      pcan_data_trasmit( PLIN_USB_EP_CMDIN, pcmd_resp, plin_packet_size( pcmd->id  ) );
      break;
    case PLIN_USB_CMD_GET_MODE:
      pcmd_resp->get_mode.mode = lin_device.lin[pcmd->bus].mode;
      pcmd_resp->get_mode.unused[0] = 0;
      pcmd_resp->get_mode.unused[1] = 0;
      pcmd_resp->get_mode.unused[2] = 0;
      pcan_data_trasmit( PLIN_USB_EP_CMDIN, pcmd_resp, plin_packet_size( pcmd->id  ) );
      break;
    case PLIN_USB_CMD_SET_IDENT_STR:
      memcpy( lin_device.lin[pcmd->bus].ident.str, pcmd->ident_str.str, sizeof( lin_device.lin[pcmd->bus].ident ) );
      break;
    case PLIN_USB_CMD_GET_HW_IDENT:
#if ( PCAN_PRO_FD == 1 )
        pcmd_resp->ident_str.str[0] = LIN_HW_TYPE_USB_PRO_FD;
#elif ( PCAN_PRO == 1 )
        pcmd_resp->ident_str.str[0] = LIN_HW_TYPE_USB_PRO;
#else
        pcmd_resp->ident_str.str[0] = LIN_HW_TYPE_PLIN_USB;
#endif
      pcmd_resp->ident_str.str[1] = 0x00;
      pcmd_resp->ident_str.str[2] = 0x00;
      pcmd_resp->ident_str.str[3] = 0x00;

      pcan_data_trasmit( PLIN_USB_EP_CMDIN, pcmd_resp, 4 + 4  );
      break;
    case PLIN_USB_CMD_GET_IDENT_STR:
      memcpy( pcmd_resp->ident_str.str, lin_device.lin[pcmd->bus].ident.str, sizeof( lin_device.lin[pcmd->bus].ident ) );
      pcan_data_trasmit( PLIN_USB_EP_CMDIN, pcmd_resp, plin_packet_size( pcmd->id  ) );
      break;
    /* LED blinking to indentify this bus */
    case PLIN_USB_CMD_IDENTIFY_BUS:
      pcan_led_set_mode( pcmd->bus ? LED_LIN_LED1:LED_LIN_LED0, LED_MODE_BLINK_FAST, 200 );
      break;
    case PLIN_USB_CMD_GET_FW_VER:
      pcmd_resp->fw_ver.major = 3;
      pcmd_resp->fw_ver.minor = 2;
      pcmd_resp->fw_ver.sub = 1;
      pcan_data_trasmit( PLIN_USB_EP_CMDIN, pcmd_resp, plin_packet_size( pcmd->id  ) );
      break;
    /* TODO: keep alive message */
    /* keep alive message can be used only while data scheduler is not used */
    case PLIN_USB_CMD_START_KEEP_ALIVE:
      lin_device.lin[pcmd->bus].keep_alive = pcmd->keep_alive;
      lin_device.lin[pcmd->bus].enable_ka_frame = 1;
      break;
    case PLIN_USB_CMD_RESUME_KEEP_ALIVE:
      lin_device.lin[pcmd->bus].enable_ka_frame = 1;
      break;
    case PLIN_USB_CMD_SUSPEND_KEEP_ALIVE:
      lin_device.lin[pcmd->bus].enable_ka_frame = 0;
      break;
    /* TODO: scheduler table processing */
    case PLIN_USB_CMD_ADD_SCHED_SLOT:
      break;
    case PLIN_USB_CMD_DEL_SCHED_SLOT:
      break;
    case PLIN_USB_CMD_GET_SLOTS_COUNT:
      break;
    case PLIN_USB_CMD_GET_SCHED_SLOT:
      break;
    case PLIN_USB_CMD_SET_SCHED_BRKPT:
      break;
    case PLIN_USB_CMD_START_SCHED:
      break;
    case PLIN_USB_CMD_RESUME_SCHED:
      break;
    case PLIN_USB_CMD_SUSPEND_SCHED:
      break;
    case PLIN_USB_CMD_GET_STATUS:
      pcmd_resp->get_status.mode = lin_device.lin[pcmd->bus].mode;
      pcmd_resp->get_status.tx_qfree = 16;
      pcmd_resp->get_status.schd_poolfree = 256;
      pcmd_resp->get_status.baudrate = lin_device.lin[pcmd->bus].baudrate;
      pcmd_resp->get_status.usb_rx_ovr = 0;
      pcmd_resp->get_status.usb_filter = 0x00;
      pcmd_resp->get_status.bus_state = PLIN_USB_ACTIVE;
      memset( pcmd_resp->get_status.unused, 0x0, sizeof( pcmd_resp->get_status.unused ) );

      pcan_data_trasmit( PLIN_USB_EP_CMDIN, pcmd_resp, plin_packet_size( pcmd->id  ) );
      break;
    case PLIN_USB_CMD_RESET:
      /* reinit interface ? flush TX/RX fifo ? */
      break;
    case PLIN_USB_CMD_UPDATE_BYTE_ARRAY:
      plin_lin_update_entry( pcmd->bus, &pcmd->update_data );
      break;
    case PLIN_USB_CMD_XMT_WAKE_UP:
      plin_lin_write_byte( pcmd->bus, 0x80 );
      break;
    case PLIN_USB_CMD_RSP_REMAP:
      memcpy( &pcmd_resp->rsp_remap, &pcmd->rsp_remap, sizeof( struct plin_usb_rsp_remap ) );
      plin_lin_remap_table( pcmd->bus, &pcmd_resp->rsp_remap );
      if( pcmd_resp->rsp_remap.set_get == PLIN_USB_RSP_REMAP_GET )
      {
        pcan_data_trasmit( PLIN_USB_EP_CMDIN, pcmd_resp, plin_packet_size( pcmd->id  ) );
      }
      break;
    case PLIN_USB_CMD_LED_STATE:
    {
      int state = pcmd->led_state.on_off?LED_MODE_ON:LED_MODE_OFF;
      pcan_led_set_mode( LED_STAT, state, 0 );
    }
      break;
    /* ? */
    default:
      assert( 0 );
      return;
  }
}

#define PLIN_SERVICE_HEADER_SIZE  ( 8 )
void plin_protocol_process_data( uint8_t *ptr, uint16_t size, uint8_t ep )
{
  int bus;
  struct plin_msg msg = { 0 };

  if( PLIN_USB_EP_MSGOUT_CH1 == ep )
    bus = LIN_BUS_1;
  else if( PLIN_USB_EP_MSGOUT_CH2 == ep )
    bus = LIN_BUS_2;
  else
    return;

  int req_size = PLIN_SERVICE_HEADER_SIZE + 8 + ptr[1] /* len */;
  if( size < req_size )
    return;

  /* skip some service info */
  ptr += PLIN_SERVICE_HEADER_SIZE;

  msg.type = PLIN_MSG_FRAME;
  msg.flags = 0;
  msg.id =  ptr[0]&PLIN_ID_MASK; /* protected ID => id */
  msg.len = ptr[1];
  msg.dir = ptr[2];
  msg.cs_type = ptr[3];
  /* data checksum, linux does not provide it */
  msg.reserved[0] = ptr[4];
  /* copy data set */
  memcpy( msg.data, &ptr[8], msg.len );

  pcan_led_set_mode( bus == LIN_BUS_1 ? LED_LIN_LED0:LED_LIN_LED1, LED_MODE_BLINK_FAST, 237 );
  (void)plin_lin_write( bus, &msg );
}

void plin_protocol_poll( void )
{
  struct plin_msg msg = { 0 };
  const int lin_bus_array[LIN_BUS_TOTAL] = { LIN_BUS_1, LIN_BUS_2 };

  for( int i = 0; i < LIN_BUS_TOTAL; i++ )
  {
    int bus = lin_bus_array[i];
    if( plin_lin_read( bus, &msg ) < 0 )
      continue;
    /* process message */
    pcan_led_set_mode( bus == LIN_BUS_1 ? LED_LIN_LED0:LED_LIN_LED1, LED_MODE_BLINK_FAST, 237 );
    plin_process_rx_message( bus, &msg );
  }

  if( msg_pos > 4 )
  {
    int res = pcan_flush_data( &resp_fsm[0], msg_buffer, PLIN_DATA_BUFFER_SIZE );
    if( res )
    {
      plin_buffer_reset();
    }
  }
}
