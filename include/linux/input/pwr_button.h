/**
 * linux/include/input/pwr_button.h
 *
 * Copyright (C) 2013 Truby Zong <truby.zong@gmail.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#ifndef _PWR_BUTTON_H
#define _PWR_BUTTON_H

/**
 * struct pwr_button_platform_data - platform-specific pwr_button data
 * @irq:           power button irq number
 * @delay:         delay time of power off
 * @init:          control hw initialze
 * @exit:          control hw un-initialize
 * @get_status:    obtain current up interrupt level
 * @get_power_type obtain power supply type
 */
struct pwr_button_platform_data {
    u32  irq;
    u32  delay;
    int  (*init)(void);
    void (*exit)(void);
    int  (*get_status)(void);
};

#endif /* _PWR_BUTTON_H */
