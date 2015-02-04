/*
 * Copyright:
 * ----------------------------------------------------------------
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 *   (C) COPYRIGHT 2013 ARM Limited
 *       ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 * ----------------------------------------------------------------
 * File:     icons.h
 * Release:  Version 2.0
 * ----------------------------------------------------------------
 *
 *            Color LCD Support
 *            =================
 */

#ifndef _ICONS_H
#define _ICONS_H


// Screen size
#define LCD_WIDTH           320         // Screen Width (in pixels)
#define LCD_HEIGHT          240         // Screen Height (in pixels)

// External Variables from flyer.c
#include "flyer.c"
extern const unsigned short flyerData[];
// External Variables from armtag.c
#include "armtag.c"
extern const unsigned short armtag[];
// External Variables from mbedtag.c
#include "mbedtag.c"
extern const unsigned short mbedtag[];
// External Variables from otherscreen.c
#include "otherscreen.c"
extern const unsigned short otherscreen[];
// External Variables from lines_button.c
#include "lines_button.c"
extern const unsigned short lines_button[];
// External Variables from demo_button.c
#include "demo_button.c"
extern const unsigned short demo_button[];
// External Variables from worm_button.c
#include "worm_button.c"
extern const unsigned short worm_button[];
// External Variables from other_button.c
#include "other_button.c"
extern const unsigned short other_button[];
// External Variables from mainmenu.c
#include "mainmenu.c"
extern const unsigned short mainmenu[];
// External Variables from press.c
#include "press.c"
extern const unsigned short pressData[];
// External Variables from exit.c
#include "exit.c"
extern const unsigned short exitData[];
// External Variables from slide.c
#include "slide.c"
extern const unsigned short slideData[];
// External Variables from slider.c
#include "slider.c"
extern const unsigned short sliderData[];
// External Variables from slidev.c
#include "slidev.c"
extern const unsigned short slidevData[];
// External Variables from sliderv.c
#include "sliderv.c"
extern const unsigned short slidervData[];
// External Variables from car1.c
#include "car1.c"
extern const unsigned short car1Data[];
// External Variables from car2.c
#include "car2.c"
extern const unsigned short car2Data[];
// External Variables from car3.c
#include "car3.c"
extern const unsigned short car3Data[];
// External Variables from next.c
#include "next.c"
extern const unsigned short nextData[];
// External Variables from ledon.c
#include "ledon.c"
extern const unsigned short ledonData[];
// External Variables from ledoff.c
#include "ledoff.c"
extern const unsigned short ledoffData[];
// External Variables from switch.c
#include "switch.c"
extern const unsigned short switchData[];
// External Variables from spkr.c
#include "spkr.c"
extern const unsigned short spkrData[];
// External Variables from spkroff.c
#include "spkroff.c"
extern const unsigned short spkroffData[];
// External Variables from intro.c
#include "intro.c"
extern const unsigned short textcolour[];
extern const unsigned short thecolours[];
extern const unsigned short smallredsquare[];
extern const unsigned short smallgreensquare[];
extern const unsigned short smallyellowsquare[];
extern const unsigned short smallcyansquare[];
extern const unsigned short introData[];
extern const int sinewave[];

#endif /* _ICONS_H */

