/*
 * Stepper Motor Platform Data
 *
 * Copyright 2014  Truby.Zong <truby.zong@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */
#ifndef _STEPPER_H
#define _STEPPER_H

#define MAX_STEPPER_NUM 8
struct stepper_info {
    char *name;
    int step;
    int dir;
    int fault;
};

struct stepper_platform_data {
    struct stepper_info *steppers;
    int nsteppers;

    int (*check_min_x)(void);
    int (*check_min_y)(void);
    int (*check_min_z)(void);

    int (*check_max_x)(void);
    int (*check_max_y)(void);
    int (*check_max_z)(void);
    int (*check_autoLevel_z)(void);
};

struct vref_platform_data {
    char *name;
    int def_uV;
    int min_uV;
    int max_uv;
};

#endif
