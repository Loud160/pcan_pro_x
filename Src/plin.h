/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * PEAK-System LIN Driver API for Linux
 *
 * Copyright (C) 2014-2020 PEAK System-Technik GmbH <www.peak-system.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact: <linux@peak-system.com>
 * Author:  Stephane Grosjean <s.grosjean@peak-system.com>
 */
#ifndef __PLIN_H__
#define __PLIN_H__

#include <stdint.h>

#ifndef u8
#define u8      uint8_t
#define u16     uint16_t
#define u32     uint32_t
#define u64     uint64_t
#endif
#ifndef __le16
#define __le16  uint16_t
#define __le32  uint32_t
#define __le64  uint64_t
#endif

/* LIN adapter models */
#define  LIN_HW_TYPE_USB                1
#define  LIN_HW_TYPE_USB_PRO            1
#define  LIN_HW_TYPE_USB_PRO_FD         2
#define  LIN_HW_TYPE_PLIN_USB           3

/* LIN errors */
#define PLIN_ERR_OK           0 /* success */
#define PLIN_ERR_FAIL         1 /* (PCAN-USB Pro) failure */
#define PLIN_ERR_INITIALIZE   2 /* hw initialized */
#define PLIN_ERR_SCHEDULER    3 /* scheduler bad state */
#define PLIN_ERR_FRAME        4 /* bad frame config */
#define PLIN_ERR_SLOTPOOL     5 /* slots global pool full */
#define PLIN_ERR_ILL_SCHEDULE 6 /* no schedule present */
#define PLIN_ERR_ILL_MODE     7 /* bad mode */

/* LIN mode */
#define PLIN_MODE_NONE        0
#define PLIN_MODE_SLAVE       1
#define PLIN_MODE_MASTER      2

/* LIN message type */
#define PLIN_MSG_FRAME          0
#define PLIN_MSG_SLEEP          1
#define PLIN_MSG_WAKEUP         2
#define PLIN_MSG_AUTOBAUD_TO    3
#define PLIN_MSG_AUTOBAUD_OK    4
#define PLIN_MSG_OVERRUN        5

/* LIN frame direction */
#define PLIN_FRM_DIR_DISABLED             0
#define PLIN_FRM_DIR_PUBLISHER            1
#define PLIN_FRM_DIR_SUBSCRIBER           2
#define PLIN_FRM_DIR_SUBSCRIBER_AUTO_LEN  3

/* LIN frame checksum type */
#define PLIN_FRM_CST_CUSTOM       0
#define PLIN_FRM_CST_CLASSIC      1
#define PLIN_FRM_CST_ENHANCED     2
#define PLIN_FRM_CST_AUTO         3 /* not for publisher */

/* LIN frame flags for frame table entry */
#define PLIN_FRM_FLG_RSP_ENABLE     0x0001  /* publisher */
#define PLIN_FRM_FLG_SINGLE_SHOT    0x0002  /* publisher */
#define PLIN_FRM_FLG_IGNORE_DATA    0x0004

/* LIN frame received (error) flags */
#define PLIN_FRM_ERR_INC_SYNC       0x0001
#define PLIN_FRM_ERR_PARITY0        0x0002
#define PLIN_FRM_ERR_PARITY1        0x0004
#define PLIN_FRM_ERR_SLV_NOT_RSP    0x0008
#define PLIN_FRM_ERR_TIMEOUT        0x0010
#define PLIN_FRM_ERR_BAD_CS         0x0020
#define PLIN_FRM_ERR_BUS_SHORT_GND  0x0040
#define PLIN_FRM_ERR_BUS_SHORT_VBAT 0x0080
#define PLIN_FRM_ERR_RESERVED       0x0100
#define PLIN_FRM_ERR_OTHER_RSP      0x0200

/* LIN data length */
#define PLIN_DAT_LEN                8

struct plin_msg {
  u16 type;
  u16 flags;
  u8  id;
  u8  len;
  u8  dir;
  u8  cs_type;
  u64 ts_us;
  u8  data[PLIN_DAT_LEN];
  u8  reserved[8];
};

/* LIN baudrate range */
#define PLIN_BDR_MIN        1000
#define PLIN_BDR_MAX        20000

/* LIN frame Id range */
#define PLIN_FRM_ID_MIN        0
#define PLIN_FRM_ID_MAX        63

#define PLIN_FRM_ID_UNC_MIN           0
#define PLIN_FRM_ID_UNC_MAX           59
#define PLIN_FRM_ID_DIAG_MASTER_REQ   60
#define PLIN_FRM_ID_DIAG_SLAVE_RSP    61
#define PLIN_FRM_ID_USER              62
#define PLIN_FRM_ID_RESERVED          63

/* LIN schedule max entries */
#define PLIN_SCH_IDX_MIN      0
#define PLIN_SCH_IDX_MAX      7

/*
 * PLIN USB direct API
 */
struct plin_usb_init_hw {
  __le16 baudrate;
  u8 mode;
  u8 unused;
}
__attribute__    ((packed));

struct plin_usb_frm_entry {
  u8 id;
  u8 len;
  u8 direction;
  u8 checksum;
  __le16 flags;
  __le16 unused;
  u8 d[PLIN_DAT_LEN];
}
__attribute__      ((packed));

struct plin_usb_auto_baud {
  __le16 timeout;
  u8 err;
  u8 unused;
}
__attribute__      ((packed));

struct plin_usb_get_baudrate {
  __le16 baudrate;
  __le16 unused;
}
__attribute__         ((packed));

struct plin_usb_id_filter {
  u8 id_mask[8];
}
__attribute__      ((packed));

struct plin_usb_get_mode {
  u8 mode;
  u8 unused[3];
}
__attribute__     ((packed));

struct plin_usb_ident_str {
  u8 str[48];
}
__attribute__      ((packed));

struct plin_usb_fw_ver {
  u8 major;
  u8 minor;
  __le16 sub;
}
__attribute__   ((packed));

struct plin_usb_keep_alive {
  u8 err;
  u8 id;
  __le16 period_ms;
}
__attribute__       ((packed));

#define PLIN_USB_SLOT_UNCOND      0 /* id0 is sent */
#define PLIN_USB_SLOT_EVENT       1 /* id0 is sent */
#define PLIN_USB_SLOT_SPORADIC    2 /* id0 highest priority */
#define PLIN_USB_SLOT_MASTER_REQ  3 /* 60h implicit */
#define PLIN_USB_SLOT_SLAVE_RSP   4 /* 61h implicit */

#define PLIN_USB_SLOT_NB_MIN    1
#define PLIN_USB_SLOT_NB_MAX    8

struct plin_usb_add_schd_slot {
  u8 schedule;
  u8 err;
  u16 unused;
  u8 type;  /* PLIN_USB_SLOT_xxx */
  u8 count_resolve;
  __le16 delay;
  u8 id[PLIN_USB_SLOT_NB_MAX];
  u32 handle;
}
__attribute__          ((packed));

struct plin_usb_del_schd {
  u8 schedule;
  u8 err;
  u16 unused;
}
__attribute__     ((packed));

struct plin_usb_get_slot_count {
  u8 schedule;
  u8 unused;
  __le16 count;
}
__attribute__           ((packed));

struct plin_usb_get_schd_slot {
  u8 schedule;  /* schedule from which the slot is returned */
  u8 slot_idx;  /* slot index returned */
  u8 err; /* 1: no schedule present */
  u8 unused;
  u8 type;  /* PLIN_USB_SLOT_xxx */
  u8 count_resolve;
  __le16 delay;
  u8 id[PLIN_USB_SLOT_NB_MAX];
  u32 handle;
}
__attribute__          ((packed));

struct plin_usb_set_schd_brkpt {
  u8 brkpt; /* 0/1 */
  u8 unused[3];
  u32 handle; /* slot handle returned */
}
__attribute__           ((packed));

#define plin_usb_start_schd    plin_usb_del_schd

struct plin_usb_resume_schd {
  u8 err; /* 1: not master or no schedule started before */
  u8 unused[3];
}
__attribute__        ((packed));

struct plin_usb_suspend_schd {
  u8 err;
  u8 schedule;  /* suspended schedule idx [0..7] */
  u8 unused[2];
  u32 handle; /* slot handle */
}
__attribute__         ((packed));

/* bus_state */
#define PLIN_USB_NOT_INITIALIZED 			0       /* Hardware is not initialized */
#define PLIN_USB_AUTOBAUDRATE       	1       /* Hardware is detecting the baudrate */
#define PLIN_USB_ACTIVE             	2       /* Hardware (bus) is active */
#define PLIN_USB_SLEEP              	3       /* Hardware (bus) is in sleep mode */
#define PLIN_USB_SHORT_GROUND        	6       /* Hardware (bus-line) shorted to ground */
#define PLIN_USB_VBAT_MISSING        	7       /* Hardware (USB adapter) external voltage supply missing */

struct plin_usb_get_status {
  u8 mode;
  u8 tx_qfree;
  __le16 schd_poolfree;
  __le16 baudrate;
  __le16 usb_rx_ovr;  /* USB data overrun counter */
  __le64 usb_filter;
  u8 bus_state;
  u8 unused[3];
}
__attribute__       ((packed));

struct plin_usb_update_data {
  u8 id;  /* frame id to update [0..63] */
  u8 len; /* count of data bytes to update [1..8] */
  u8 idx; /* data offset [0..7] */
  u8 unused;
  u8 d[PLIN_DAT_LEN]; /* new data bytes */
}
__attribute__        ((packed));

#define PLIN_USB_RSP_REMAP_GET      0
#define PLIN_USB_RSP_REMAP_SET      1
#define PLIN_USB_RSP_REMAP_ID_LEN   (PLIN_FRM_ID_MAX - PLIN_FRM_ID_MIN + 1)

struct plin_usb_rsp_remap {
  u8 set_get; /* PLIN_USB_RSP_REMAP_xET */
  u8 unused[3];
  u8 id[PLIN_USB_RSP_REMAP_ID_LEN];
}
__attribute__      ((packed));

#define PLIN_USB_LEDS_OFF   0
#define PLIN_USB_LEDS_ON    1

struct plin_usb_led_state {
  u8 on_off;  /* PLIN_USB_LEDS_xxx */
  u8 unused[3];
}
__attribute__      ((packed));

/*
 * chardev API: ioctls list
 */

/* USB direct API */
#define PLIOHWINIT          _IOW('u', 0, struct plin_usb_init_hw)
#define PLIORSTHW           _IO('u', 1)
#define PLIOSETFRMENTRY     _IOW('u', 2, struct plin_usb_frm_entry)
#define PLIOGETFRMENTRY     _IOWR('u', 3, struct plin_usb_frm_entry)
#define PLIOSTARTAUTOBAUD   _IOWR('u', 4, struct plin_usb_auto_baud)
#define PLIOGETBAUDRATE     _IOWR('u', 5, struct plin_usb_get_baudrate)
#define PLIOSETIDFILTER     _IOW('u', 6, struct plin_usb_id_filter)
#define PLIOGETIDFILTER     _IOWR('u', 7, struct plin_usb_id_filter)
#define PLIOGETMODE         _IOWR('u', 8, struct plin_usb_get_mode)
#define PLIOSETIDSTR        _IOW('u', 9, struct plin_usb_ident_str)
#define PLIOGETIDSTR        _IOWR('u', 10, struct plin_usb_ident_str)
#define PLIOIDENTIFY        _IO('u', 11)
#define PLIOGETFWVER        _IOWR('u', 12, struct plin_usb_fw_ver)
#define PLIOSTARTHB         _IOWR('u', 13, struct plin_usb_keep_alive)
#define PLIORESUMEHB        _IOWR('u', 14, struct plin_usb_keep_alive)
#define PLIOPAUSEHB         _IOW('u', 15, struct plin_usb_keep_alive)
#define PLIOADDSCHDSLOT     _IOWR('u', 16, struct plin_usb_add_schd_slot)
#define PLIODELSCHD         _IOWR('u', 17, struct plin_usb_del_schd)
#define PLIOGETSLOTSCNT     _IOWR('u', 18, struct plin_usb_get_slot_count)
#define PLIOGETSCHDSLOT     _IOWR('u', 19, struct plin_usb_get_schd_slot)
#define PLIOSETSCHDBP       _IOWR('u', 20, struct plin_usb_set_schd_brkpt)
#define PLIOSTARTSCHD       _IOWR('u', 21, struct plin_usb_start_schd)
#define PLIORESUMESCHD      _IOWR('u', 22, struct plin_usb_resume_schd)
#define PLIOPAUSESCHD       _IOWR('u', 23, struct plin_usb_suspend_schd)
#define PLIOGETSTATUS       _IOWR('u', 24, struct plin_usb_get_status)
#define PLIORSTUSBTX        _IO('u', 30)
#define PLIOCHGBYTEARRAY    _IOW('u', 31, struct plin_usb_update_data)
#define PLIOXMTWAKEUP       _IO('u', 33)
#define PLIOSETGETRSPMAP    _IOWR('u', 39, struct plin_usb_rsp_remap)
#define PLIOSETLEDSTATE     _IOW('u', 40, struct plin_usb_led_state)

#endif
