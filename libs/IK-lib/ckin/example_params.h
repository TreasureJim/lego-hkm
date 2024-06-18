/*
 * Copyright (C) 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
/*!
 * @file
 * @brief Example parameters, nominal.
 */
#ifndef TEST_PARAMS_H
#define TEST_PARAMS_H

#include "lpoe_invkin.h"

extern struct model_lpoe_nom ER6;
extern struct model_lpoe_nom ER6_offs;
extern struct model_lpoe_nom KR20R1810;
extern struct model_lpoe_nom KR300R2500;
extern struct model_lpoe_nom ER300;
extern struct model_lpoe_nom Smart5NJ42202_7;
extern struct model_lpoe_nom Smart5NJ42202_7_2;
extern struct model_lpoe_nom NJ130;
extern struct model_lpoe_nom NJ130_2;
extern struct model_lpoe_nom UR10e;

void init_er6();
void init_kr20r1810();
void init_kr300r2500();
void init_er300();
void init_smart5nj42202_7();
void init_nj130();

#endif
