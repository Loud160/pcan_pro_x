#include <assert.h>
#include <string.h>
#include "io_macro.h"
#if defined( STM32G4 )
#include <stm32g4xx_ll_usart.h>
#elif defined( STM32F4 )
#include <stm32f4xx_ll_usart.h>
#endif
#include "pcanpro_timestamp.h"
#include "plin_lin.h"

#define UART1_RX  A, 10, MODE_AF_OD, PULLUP, SPEED_FREQ_VERY_HIGH, AF7_USART1
#define UART1_TX  A, 9, MODE_AF_PP, NOPULL, SPEED_FREQ_VERY_HIGH, AF7_USART1

#if defined( STM32G4 )
#define UART2_RX  B, 4, MODE_AF_OD, PULLUP, SPEED_FREQ_VERY_HIGH, AF7_USART2
#define UART2_TX  B, 3, MODE_AF_PP, NOPULL, SPEED_FREQ_VERY_HIGH, AF7_USART2
#elif defined( STM32F4 )
#define UART2_RX  D, 6, MODE_AF_OD, PULLUP, SPEED_FREQ_VERY_HIGH, AF7_USART2
#define UART2_TX  D, 5, MODE_AF_PP, NOPULL, SPEED_FREQ_VERY_HIGH, AF7_USART2
#endif

#define LIN1_MASTER   A, 1, MODE_OUTPUT_OD, NOPULL, SPEED_FREQ_VERY_HIGH, NOAF
#define LIN1_SLEEP    A, 0, MODE_OUTPUT_PP, NOPULL, SPEED_FREQ_VERY_HIGH, NOAF

#define LIN2_MASTER   A, 5, MODE_OUTPUT_OD, NOPULL, SPEED_FREQ_VERY_HIGH, NOAF
#define LIN2_SLEEP    A, 4, MODE_OUTPUT_PP, NOPULL, SPEED_FREQ_VERY_HIGH, NOAF

static USART_TypeDef *hlin[LIN_BUS_TOTAL] = 
{ 
  [LIN_BUS_1] = USART1,
  [LIN_BUS_2] = USART2,
};

static const IRQn_Type hlin_irqn[LIN_BUS_TOTAL] = 
{
  [LIN_BUS_1] = USART1_IRQn,
  [LIN_BUS_2] = USART2_IRQn
};

/* RX */
enum
{
  LIN_WAIT_BREAK,
  LIN_WAIT_SYNC,
  LIN_WAIT_PID ,
  LIN_WAIT_DATA,
  LIN_WAIT_CRC,
  LIN_RESP_DATA,
  LIN_RESP_CRC,
  LIN_RESP_DROP,
};

/* TX */
enum
{
  LIN_SEND_BREAK,
  LIN_SEND_SYNC,
  LIN_SEND_PID ,
  LIN_SEND_DATA,
  LIN_SEND_CRC,
};

/* BUSY FLAGS */
enum
{
  LIN_READY_TXRX,
  LIN_BUSY_TX,
  LIN_BUSY_RX,
};

#define LIN_SYNC_BYTE       (0x55)
#define PLIN_FIFO_SIZE      (16)
#define PLIN_ID_MAX_COUNT   (PLIN_FRM_ID_MAX - PLIN_FRM_ID_MIN + 1)

struct t_plin_fifo
{
  struct plin_msg msg[PLIN_FIFO_SIZE];
  uint8_t   head;
  uint8_t   tail;
  uint8_t   state;
  uint8_t   pos;
  uint8_t   crc;
  uint32_t  ts;
};

static struct t_plin_device
{
  uint8_t                   is_busy;
  uint8_t                   mode;
  uint32_t                  guard_time;
  struct t_plin_fifo        tx_fifo;
  struct t_plin_fifo        rx_fifo;
  /* ID accept mask */
  uint8_t                   filter_mask[8];
  uint8_t                   remap_table[PLIN_ID_MAX_COUNT];
  struct plin_usb_frm_entry table[PLIN_ID_MAX_COUNT];
}
plin_device[LIN_BUS_TOTAL] = 
{
  0
};

#define PLIN_ID_MASK (PLIN_FRM_ID_MAX)
static uint8_t lin_parity( uint8_t d )
{
  uint8_t p0, p1;

  p0 =  (d>>0)&1;
  p0 ^= (d>>1)&1;
  p0 ^= (d>>2)&1;
  p0 ^= (d>>4)&1;

  p1 = 1;
  p1 ^= (d>>1)&1;
  p1 ^= (d>>3)&1;
  p1 ^= (d>>4)&1;
  p1 ^= (d>>5)&1;

  return ( d & PLIN_ID_MASK ) | (p1<<7) | (p0<<6);
}

static uint8_t lin_frame_size( uint8_t d )
{
  static const uint8_t size_array[4] = { 2, 2, 4, 8 };

  return size_array[(d>>4)&0x03];
}

static uint8_t lin_update_crc( uint8_t seed, uint8_t d )
{
  uint16_t r = seed + d;
  if( r > 0xFF )
    r -= 0xFF;

  return r & 0xFF;
 }

int plin_lin_init( void )
{
  for( int i = 0; i < LIN_BUS_TOTAL; i++ )
  {
    struct plin_usb_frm_entry *ptable = &plin_device[i].table[0];
    for( int n = 0; n < PLIN_ID_MAX_COUNT; n++ )
    {
      ptable[n].id = n;
      ptable[n].len = lin_frame_size( n );
      ptable[n].direction = PLIN_FRM_DIR_SUBSCRIBER_AUTO_LEN;
      ptable[n].checksum = PLIN_FRM_CST_AUTO;
      ptable[n].flags = 0;
    }

    for( int n = 0; n < PLIN_ID_MAX_COUNT; n++ )
    {
      plin_device[i].remap_table[n] = n;
    }
    /* reset fifo states */
    plin_device[i].rx_fifo.state = LIN_WAIT_BREAK;
    plin_device[i].tx_fifo.state = LIN_SEND_BREAK;
  }

  return 0;
}

int plin_lin_remap_table( int bus, struct plin_usb_rsp_remap *ptable )
{
  assert( bus < LIN_BUS_TOTAL );

  if( ptable->set_get == PLIN_USB_RSP_REMAP_SET )
    memcpy( plin_device[bus].remap_table, ptable->id, PLIN_ID_MAX_COUNT );
  else
    memcpy( ptable->id, plin_device[bus].remap_table, PLIN_ID_MAX_COUNT );

  return 0;
}

int plin_lin_get_tentry( int bus, struct plin_usb_frm_entry *p )
{
  assert( bus < LIN_BUS_TOTAL );

  *p = plin_device[bus].table[p->id&PLIN_ID_MASK];
  return 0;
}

int plin_lin_set_tentry( int bus, struct plin_usb_frm_entry *p )
{
  assert( bus < LIN_BUS_TOTAL );

  plin_device[bus].table[p->id&PLIN_ID_MASK] = *p;
  return 0;
}

int plin_lin_set_filter( int bus, uint8_t *pmask )
{
  assert( bus < LIN_BUS_TOTAL );

  memcpy( plin_device[bus].filter_mask, pmask, sizeof( plin_device[bus].filter_mask ) );
  return 0;
}

int plin_lin_get_filter( int bus, uint8_t *pmask )
{
  assert( bus < LIN_BUS_TOTAL );

  memcpy( pmask, plin_device[bus].filter_mask, sizeof( plin_device[bus].filter_mask ) );
  return 0;
}

int plin_lin_update_entry( int bus, struct plin_usb_update_data *pdata )
{
  assert( bus < LIN_BUS_TOTAL );

  struct plin_usb_frm_entry *ptable = &plin_device[bus].table[pdata->id&PLIN_ID_MASK];

  for( int n = 0; n < pdata->len; n++ )
  {
    int offset = ( pdata->idx + n ) & 7;
    ptable->d[offset] = pdata->d[offset];
  }

  return 1;
}

int plin_lin_disable( int bus )
{
  USART_TypeDef *pdev = hlin[bus];

  NVIC_DisableIRQ( hlin_irqn[bus] );
  LL_USART_Disable( pdev );
#if defined( STM32G4 )
  pdev->ICR = 0xFFFFFFFF;
#endif

  plin_lin_set_mode( bus, PLIN_MODE_NONE );

  return 0;
}

int plin_lin_init_ex( int bus, uint32_t bitrate, uint8_t mode )
{
  assert( bus < LIN_BUS_TOTAL );

  USART_TypeDef *pdev = hlin[bus];
  LL_USART_InitTypeDef config;

  LL_USART_StructInit( &config );

  switch( bus )
  {
    case LIN_BUS_1:
      PORT_ENABLE_CLOCK( PIN_PORT( LIN1_MASTER ), PIN_PORT( LIN1_SLEEP ) );
      PORT_ENABLE_CLOCK( PIN_PORT( UART1_RX ), PIN_PORT( UART1_TX ) );
      PIN_HI( LIN1_SLEEP );
      PIN_LOW( LIN1_MASTER );
      PIN_INIT( LIN1_SLEEP );
      PIN_INIT( LIN1_MASTER );

      __HAL_RCC_USART1_CLK_ENABLE();
      PIN_INIT( UART1_RX );
      PIN_INIT( UART1_TX );
    break;
    case LIN_BUS_2:
      PORT_ENABLE_CLOCK( PIN_PORT( LIN2_MASTER ), PIN_PORT( LIN2_SLEEP ) );
      PORT_ENABLE_CLOCK( PIN_PORT( UART2_RX ), PIN_PORT( UART2_TX )  );
      PIN_HI( LIN2_SLEEP );
      PIN_LOW( LIN2_MASTER );
      PIN_INIT( LIN2_SLEEP );
      PIN_INIT( LIN2_MASTER );

      __HAL_RCC_USART2_CLK_ENABLE();
      PIN_INIT( UART2_RX );
      PIN_INIT( UART2_TX );
    break;
  }
  /* disable LIN */
  if( mode == PLIN_MODE_NONE )
  {
    return plin_lin_disable( bus );
  }

  /* set guard time to 32 bit */
  plin_device[bus].guard_time = (1000000u/bitrate)*32;
  /* update node mode */
  plin_device[bus].mode = mode;

  LL_USART_DeInit( pdev );

#if defined( STM32G4 )
  config.PrescalerValue = LL_USART_PRESCALER_DIV1;
#endif
  config.BaudRate = bitrate;
  config.DataWidth = LL_USART_DATAWIDTH_8B;
  config.StopBits = LL_USART_STOPBITS_1;
  config.Parity = LL_USART_PARITY_NONE;
  config.TransferDirection = LL_USART_DIRECTION_TX_RX;
  config.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  config.OverSampling = LL_USART_OVERSAMPLING_16;

  LL_USART_Init( pdev, &config );

#if defined( STM32G4 )
  LL_USART_ConfigFIFOsThreshold( pdev, LL_USART_FIFOTHRESHOLD_1_8, LL_USART_FIFOTHRESHOLD_1_8 );
  LL_USART_DisableFIFO( pdev );
#endif

  plin_lin_set_mode( bus, mode );

  LL_USART_EnableIT_RXNE( pdev );
  /* use to determinate frame holes */
  LL_USART_EnableIT_IDLE( pdev );
  
  LL_USART_EnableLIN( pdev );
  LL_USART_SetLINBrkDetectionLen( pdev, LL_USART_LINBREAK_DETECT_11B );
  LL_USART_EnableIT_LBD( pdev );

  NVIC_EnableIRQ( hlin_irqn[bus] );
  LL_USART_Enable( pdev );
  return 0;
}

void plin_lin_set_mode( int bus, uint8_t mode )
{
  assert( bus < LIN_BUS_TOTAL );

  switch( bus )
  {
    case LIN_BUS_1:
      if( mode == PLIN_MODE_MASTER )
        PIN_HI( LIN1_MASTER );
      else
        PIN_LOW( LIN1_MASTER );
    break;
    case LIN_BUS_2:
      if( mode == PLIN_MODE_MASTER )
        PIN_HI( LIN2_MASTER );
      else
        PIN_LOW( LIN2_MASTER );
    break;
  }
}

static int lin_fifo_update_head( struct t_plin_fifo *p )
{
  uint8_t head_next = (p->head+1)&(PLIN_FIFO_SIZE-1);
  /* no free space ? drop it ! */
  if( head_next == p->tail )
    return -1;
  p->head = head_next;

  return 0;
}

static void lin_msg_init( struct plin_msg *pmsg )
{
  pmsg->type = PLIN_MSG_FRAME;
  pmsg->flags = 0;
  pmsg->id = 0;
  pmsg->len = 0;
  pmsg->dir = PLIN_FRM_DIR_DISABLED;
  pmsg->cs_type = PLIN_FRM_CST_CLASSIC;
  pmsg->ts_us = pcan_timestamp_us();
}

static void lin_isr_rx_handler( int bus, uint8_t data )
{
  USART_TypeDef *pdev = hlin[bus];
  struct t_plin_device *plin = &plin_device[bus];
  struct t_plin_fifo  *pfifo = &plin->rx_fifo;
  struct plin_msg *pmsg = &pfifo->msg[pfifo->head];

  uint32_t ts = pcan_timestamp_us();

  if( ( ts - pfifo->ts ) > plin->guard_time )
  {
    plin->is_busy = ( plin->is_busy == LIN_BUSY_RX ) ? LIN_READY_TXRX: plin->is_busy;
    pfifo->state = LIN_WAIT_BREAK;
  }
  pfifo->ts = ts;

  switch( pfifo->state )
  {
    default:
      plin->is_busy = ( plin->is_busy == LIN_BUSY_RX ) ? LIN_READY_TXRX: plin->is_busy;
      pfifo->state = LIN_WAIT_BREAK;
    break;
    case LIN_WAIT_SYNC:
      if( data != LIN_SYNC_BYTE )
        break;
      /* take ownership */
      if( plin->is_busy == LIN_READY_TXRX )
      {
        plin->is_busy = LIN_BUSY_RX;
      }
      pfifo->state = LIN_WAIT_PID;
    break;
    case LIN_WAIT_PID:
    {
      uint8_t dir;
      lin_msg_init( pmsg );

      if( lin_parity( data ) != data ) /* check PID parity, stop RX data if fail ? */
      {
        /* determinate invalid parity bits */
        uint8_t error_bits = lin_parity( data ) ^ data;
        if( error_bits & (1<<6) )
          pmsg->flags |= PLIN_FRM_ERR_PARITY0;
        if( error_bits & (1<<7) )
          pmsg->flags |= PLIN_FRM_ERR_PARITY1;
        /* inform driver about new frame */
        lin_fifo_update_head( pfifo );
        pfifo->state = LIN_WAIT_BREAK;
        break;
      }

      pmsg->id = lin_parity( plin->remap_table[data&PLIN_ID_MASK] ); /* PID from remap table */
      dir = plin->table[pmsg->id&PLIN_ID_MASK].direction;

      pfifo->crc = 0;
      pfifo->pos = 0;

      pmsg->len = plin->table[pmsg->id&PLIN_ID_MASK].len;
      pmsg->cs_type = plin->table[pmsg->id&PLIN_ID_MASK].checksum;
      pmsg->dir = dir;
      pmsg->ts_us = ts;

      /* SLAVE/MASTER can use autolen feature */
      if( dir == PLIN_FRM_DIR_SUBSCRIBER_AUTO_LEN )
      {
        /* set max len by default */
        pmsg->len = PLIN_DAT_LEN;
        pfifo->state = LIN_WAIT_DATA;
        break;
      }

      if( ( plin->mode == PLIN_MODE_SLAVE ) && ( dir == PLIN_FRM_DIR_PUBLISHER ) )
      {
        pfifo->state = LIN_RESP_DATA;
      }
      else
      {
        pfifo->state = LIN_WAIT_DATA;
        break;
      }
    }
    case LIN_RESP_DATA:
      data = plin->table[pmsg->id&PLIN_ID_MASK].d[pfifo->pos];
      LL_USART_TransmitData8( pdev, data );
    /* fall through */
    case LIN_WAIT_DATA:
      pmsg->data[pfifo->pos++] = data;
      pfifo->crc = lin_update_crc( pfifo->crc, data );
      if( pfifo->pos == pmsg->len )
        pfifo->state = ( pfifo->state == LIN_WAIT_DATA )? LIN_WAIT_CRC: LIN_RESP_CRC;
    break;
    case LIN_RESP_CRC:
    {
      pfifo->state = LIN_RESP_DROP;

      switch( pmsg->cs_type )
      {
        default:
        case PLIN_FRM_CST_CUSTOM:
          data = 0;
        break;
        case PLIN_FRM_CST_AUTO:
        case PLIN_FRM_CST_ENHANCED:
          data = lin_update_crc( pfifo->crc, lin_parity( pmsg->id ) );
          data ^= 0xFF;
        break;
        case PLIN_FRM_CST_CLASSIC:
          data = pfifo->crc;
          data ^= 0xFF;
        break;
      }
      LL_USART_TransmitData8( pdev, data );
    }
    break;
    case LIN_RESP_DROP:
    case LIN_WAIT_CRC:
    {
      uint8_t ex_crc;
      ex_crc = lin_update_crc( pfifo->crc, lin_parity( pmsg->id ) );
      ex_crc ^= 0xFF;
      switch( pmsg->cs_type )
      {
        case PLIN_FRM_CST_AUTO:
          if( data == ex_crc )
          {
            pfifo->crc = ex_crc;
            break;
          }
        /* fall through */
        case PLIN_FRM_CST_CLASSIC:
          pfifo->crc ^= 0xFF;
        break;
        case PLIN_FRM_CST_ENHANCED:
          pfifo->crc = ex_crc;
        break;
        case PLIN_FRM_CST_CUSTOM:
          pfifo->crc = data; /* any */
        break;
      }

      pfifo->state = LIN_WAIT_BREAK;
      if( data != pfifo->crc )
      {
        pmsg->flags |= PLIN_FRM_ERR_BAD_CS;
      }

      /* update ID to protected ID */
      pmsg->id = lin_parity( pmsg->id );
      pmsg->reserved[0] = pfifo->crc;

      if( pmsg->dir == PLIN_FRM_DIR_DISABLED )
        break;
      /* accept by filter ? */
      data = pmsg->id&PLIN_ID_MASK;
      if( ( plin->filter_mask[data>>3] & (1<<(data&7)) ) == 0 )
        break;
      (void)lin_fifo_update_head( pfifo );
    }
    break;
  }
}

static void lin_isr_tx_handler( int bus )
{
  USART_TypeDef *pdev = hlin[bus];
  struct t_plin_device *plin = &plin_device[bus];
  struct t_plin_fifo  *pfifo = &plin->tx_fifo;
  struct plin_msg *pmsg = &pfifo->msg[pfifo->tail];

  if( plin->is_busy == LIN_BUSY_RX || ( pfifo->tail == pfifo->head ) )
  {
    LL_USART_DisableIT_TXE( pdev );
    return;
  }
  plin->is_busy = LIN_BUSY_TX;

  switch( pfifo->state )
  {
    case LIN_SEND_BREAK:
      LL_USART_RequestBreakSending( pdev );
      pfifo->state = LIN_SEND_SYNC;
    break;
    case LIN_SEND_SYNC:
      LL_USART_TransmitData8( pdev, LIN_SYNC_BYTE );
      pfifo->state = LIN_SEND_PID;
    break;
    case LIN_SEND_PID:
      LL_USART_TransmitData8( pdev, lin_parity( pmsg->id ) );
      pfifo->pos = 0;
      pfifo->crc = 0;

      if( ( pmsg->dir == PLIN_FRM_DIR_SUBSCRIBER ) || ( pmsg->dir == PLIN_FRM_DIR_SUBSCRIBER_AUTO_LEN ) )
      {
        /* reset tx and wait for slave response ? */
        pfifo->state = LIN_SEND_BREAK;
        plin->is_busy = LIN_READY_TXRX;
        pfifo->tail = ( pfifo->tail + 1) & ( PLIN_FIFO_SIZE -1 );
        LL_USART_DisableIT_TXE( pdev );
      }
      else
      {
        pfifo->state = LIN_SEND_DATA;
      }
    break;
    case LIN_SEND_DATA:
      pfifo->crc = lin_update_crc( pfifo->crc, pmsg->data[pfifo->pos] );
      LL_USART_TransmitData8( pdev, pmsg->data[pfifo->pos++] );
      if( pfifo->pos == pmsg->len )
      {
        pfifo->state = LIN_SEND_CRC;
      }
    break;
    case LIN_SEND_CRC:
    {
      uint8_t data;
      
      pfifo->state = LIN_SEND_BREAK;
      LL_USART_DisableIT_TXE( pdev );

      switch( pmsg->cs_type )
      {
        default:
        case PLIN_FRM_CST_CUSTOM:
          /* ???? */
          data = 0;
        break;
        case PLIN_FRM_CST_AUTO:
        case PLIN_FRM_CST_ENHANCED:
          data = lin_update_crc( pfifo->crc, lin_parity( pmsg->id ) );
          data ^= 0xFF;
        break;
        case PLIN_FRM_CST_CLASSIC:
          pfifo->crc ^= 0xFF;
          data = pfifo->crc;
        break;
      }
      LL_USART_TransmitData8( pdev, data );
      plin->is_busy = LIN_READY_TXRX;

      pfifo->tail = ( pfifo->tail + 1) & ( PLIN_FIFO_SIZE -1 );
    }
    break;
  }
}

/* master auto len detection/slave not respond/others */
static int lin_msg_timeout( int bus )
{
  struct t_plin_device *plin = &plin_device[bus];
  struct t_plin_fifo  *pfifo = &plin->rx_fifo;
  struct plin_msg *pmsg = &pfifo->msg[pfifo->head];

  /* no pending message ? */
  if( pfifo->state == LIN_WAIT_BREAK )
    return 0;
  /* sync byte does not come ? */
  if( pfifo->state == LIN_WAIT_SYNC )
  {
    pmsg->flags |= PLIN_FRM_ERR_INC_SYNC;
    lin_fifo_update_head( pfifo );
    return 0;
  }
  
  if( !pfifo->pos )
  {
    if( plin->mode == PLIN_MODE_MASTER )
      pmsg->flags |= PLIN_FRM_ERR_SLV_NOT_RSP;
    else
      pmsg->flags |= PLIN_FRM_ERR_OTHER_RSP;
    lin_fifo_update_head( pfifo );
    return 0;
  }

  /* we asume, what minimal data frame cannpt be less than 2 bytes ( 1 data, 1 crc ) */
  if( pmsg->dir != PLIN_FRM_DIR_SUBSCRIBER_AUTO_LEN || pfifo->pos < 2 )
  {
    pmsg->flags |= PLIN_FRM_ERR_TIMEOUT;
    lin_fifo_update_head( pfifo );
    return 0;
  }

  /* try to determinate frame len, latest byte is crc */
  pfifo->crc = 0;
  pmsg->len = pfifo->pos-1;
  for( int i = 0; i < pmsg->len; i++ )
  {
    pfifo->crc = lin_update_crc( pfifo->crc, pmsg->data[i] );
  }
  /* simulate crc processing checking */
  pfifo->state = LIN_WAIT_CRC;
  lin_isr_rx_handler( bus, pmsg->data[pmsg->len] );

  return 0;
}

static void lin_isr_hadnler( int bus )
{
  USART_TypeDef *pdev = hlin[bus];
  struct t_plin_device *plin = &plin_device[bus];

  if( LL_USART_IsActiveFlag_ORE( pdev ) || LL_USART_IsActiveFlag_FE( pdev ) )
  {
    /* resume rx and resync */
    LL_USART_ClearFlag_ORE( pdev );
    LL_USART_ClearFlag_FE( pdev );
    plin->is_busy = ( plin->is_busy == LIN_BUSY_RX ) ? LIN_READY_TXRX: plin->is_busy;
    plin->rx_fifo.state = LIN_WAIT_BREAK;
  }

  if( LL_USART_IsActiveFlag_LBD( pdev ) )
  {
    LL_USART_ClearFlag_LBD( pdev );
    plin->is_busy = ( plin->is_busy == LIN_BUSY_RX ) ? LIN_READY_TXRX: plin->is_busy;
    /* break condition detected, wait for sync byte */
    plin->rx_fifo.state = LIN_WAIT_SYNC;
    plin->rx_fifo.ts = pcan_timestamp_us();
  }

  if( LL_USART_IsActiveFlag_IDLE( pdev ) )
  {
    LL_USART_ClearFlag_IDLE( pdev );
    plin->is_busy = ( plin->is_busy == LIN_BUSY_RX ) ? LIN_READY_TXRX: plin->is_busy;

    lin_msg_timeout( bus );
    plin->rx_fifo.state = LIN_WAIT_BREAK;
  }

#if defined( STM32G4 )
  if( LL_USART_IsActiveFlag_RXNE_RXFNE( pdev ) )
#elif defined( STM32F4 )
  if( LL_USART_IsActiveFlag_RXNE( pdev ) )
#else
  assert( 0 );
#endif
  {
    lin_isr_rx_handler( bus, LL_USART_ReceiveData8( pdev ) );
  }

#if defined( STM32G4 )
  if( LL_USART_IsActiveFlag_TXE_TXFNF( pdev ) )
#elif defined( STM32F4 )
  if( LL_USART_IsActiveFlag_TXE( pdev ) )
#else
  assert( 0 );
#endif
  {
    lin_isr_tx_handler( bus );
  }
}

int plin_lin_read( int bus, struct plin_msg *p_msg )
{
  struct t_plin_device *plin = &plin_device[bus];
  struct t_plin_fifo  *pfifo = &plin->rx_fifo;

  if( pfifo->head == pfifo->tail )
    return -1;

  *p_msg = pfifo->msg[pfifo->tail];
  pfifo->tail = (pfifo->tail+1)&(PLIN_FIFO_SIZE-1);

  return 0;
}

int plin_lin_write_byte( int bus, uint8_t d )
{
  USART_TypeDef *pdev = hlin[bus];

  assert( bus < LIN_BUS_TOTAL );

#if defined( STM32G4 )
  if( LL_USART_IsActiveFlag_TXE_TXFNF( pdev ) )
#elif defined( STM32F4 )
  if( LL_USART_IsActiveFlag_TXE( pdev ) )
#else
  #error TX not empty...
#endif
  {
    LL_USART_TransmitData8( pdev, d );
  }
  return 0;
}

int plin_lin_write( int bus, struct plin_msg *p_msg )
{
  USART_TypeDef *pdev = hlin[bus];
  struct t_plin_device *plin = &plin_device[bus];
  struct t_plin_fifo  *pfifo = &plin->tx_fifo;

  assert( bus < LIN_BUS_TOTAL );

  if( !p_msg )
    return -1;

  uint8_t head_next = (pfifo->head+1)&(PLIN_FIFO_SIZE-1);
  /* overflow ? sorry... */
  if( head_next == pfifo->tail )
  {
    return -1;
  }
  pfifo->msg[pfifo->head] = *p_msg;
  pfifo->head = head_next;

  /* enable sender */
  LL_USART_EnableIT_TXE( pdev );
  return 0;
}

void USART1_IRQHandler(void)
{
  lin_isr_hadnler( LIN_BUS_1 );
}

void USART2_IRQHandler(void)
{
  lin_isr_hadnler( LIN_BUS_2 );
}
