/*
 * Copyright (C) 2020 PEAK System-Technik GmbH
 * Author: Stephane Grosjean <s.grosjean@peak-system.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __PLIN_USB_H__
#define __PLIN_USB_H__

#include <stdint.h>

#define PLIN_USB_CMD_INIT_HW              0
#define PLIN_USB_CMD_RST_HW_CFG           1
#define PLIN_USB_CMD_SET_FRM_ENTRY        2
#define PLIN_USB_CMD_GET_FRM_ENTRY        3
#define PLIN_USB_CMD_START_AUTO_BAUD      4
#define PLIN_USB_CMD_GET_BAUDRATE         5
#define PLIN_USB_CMD_SET_ID_FILTER        6
#define PLIN_USB_CMD_GET_ID_FILTER        7
#define PLIN_USB_CMD_GET_MODE             8
#define PLIN_USB_CMD_SET_IDENT_STR        9
#define PLIN_USB_CMD_GET_IDENT_STR        10 
#define PLIN_USB_CMD_IDENTIFY_BUS         11
#define PLIN_USB_CMD_GET_FW_VER           12
#define PLIN_USB_CMD_START_KEEP_ALIVE     13
#define PLIN_USB_CMD_RESUME_KEEP_ALIVE    14
#define PLIN_USB_CMD_SUSPEND_KEEP_ALIVE   15
#define PLIN_USB_CMD_ADD_SCHED_SLOT       16
#define PLIN_USB_CMD_DEL_SCHED_SLOT       17
#define PLIN_USB_CMD_GET_SLOTS_COUNT      18
#define PLIN_USB_CMD_GET_SCHED_SLOT       19
#define PLIN_USB_CMD_SET_SCHED_BRKPT      20
#define PLIN_USB_CMD_START_SCHED          21
#define PLIN_USB_CMD_RESUME_SCHED         22
#define PLIN_USB_CMD_SUSPEND_SCHED        23
#define PLIN_USB_CMD_GET_STATUS           24
#define PLIN_USB_CMD_RESET                30
#define PLIN_USB_CMD_UPDATE_BYTE_ARRAY    31
#define PLIN_USB_CMD_XMT_WAKE_UP          33
#define PLIN_USB_CMD_GET_HW_IDENT         36
#define PLIN_USB_CMD_RSP_REMAP            39
#define PLIN_USB_CMD_LED_STATE            40

#define PLIN_USB_ERR_SUCCESS              0

struct plin_usb_cmd {
 uint8_t id;
 uint8_t device;
 uint8_t bus;
 uint8_t client;

 union {
  struct plin_usb_init_hw init_hw;
  struct plin_usb_frm_entry frm_entry;
  struct plin_usb_auto_baud auto_baud;
  struct plin_usb_get_baudrate get_baudrate;
  struct plin_usb_id_filter id_filter;
  struct plin_usb_get_mode get_mode;
  struct plin_usb_ident_str ident_str;
  struct plin_usb_fw_ver fw_ver;
  struct plin_usb_keep_alive keep_alive;
  struct plin_usb_add_schd_slot add_schd_slot;
  struct plin_usb_add_schd_slot del_schd_slot;
  struct plin_usb_get_slot_count get_slot_count;
  struct plin_usb_get_schd_slot get_schd_slot;
  struct plin_usb_set_schd_brkpt set_schd_brkpt;
  struct plin_usb_start_schd start_schd;
  struct plin_usb_resume_schd resume_schd;
  struct plin_usb_suspend_schd suspend_schd;
  struct plin_usb_get_status get_status;
  struct plin_usb_update_data update_data;
  struct plin_usb_rsp_remap rsp_remap;
  struct plin_usb_led_state led_state;
  uint16_t u16_raw;
  uint32_t u32_raw;
  uint8_t  p[0];
 };
} __attribute__    ((packed));

#endif
