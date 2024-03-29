/*
Copyright (c) 2012-2013, Politecnico di Milano. All rights reserved.

Andrea Zoppi <texzk@email.it>
Martino Migliavacca <martino.migliavacca@gmail.com>

http://airlab.elet.polimi.it/
http://www.openrobots.com/

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file    app.c
 * @author  Andrea Zoppi <texzk@email.it>
 *
 * @brief   Application source header.
 */

#ifndef _APP_H_
#define _APP_H_

/*===========================================================================*/
/* HEADER FILES                                                              */
/*===========================================================================*/

#include <urosBase.h>
#include <urosUser.h>
#include <urosNode.h>
#include "freertos/FreeRTOS.h"

/*===========================================================================*/
/* TYPES & MACROS                                                            */
/*===========================================================================*/

/**
 * @brief   Stream counters.
 */
typedef struct streamcnt_t {
  unsigned long numMsgs;        /**< @brief Total number of exchanged messages.*/
  size_t        numBytes;       /**< @brief Total exchanged size.*/
  unsigned long deltaMsgs;      /**< @brief Incremental number of exchanged messages.*/
  size_t        deltaBytes;     /**< @brief Incremental exchanged size.*/
} streamcnt_t;

typedef enum {
  STOP = 0,
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  ROTATE_LEFT,
  ROTATE_RIGHT,
} app_command_t;

/**
 * @brief   Benchmark status.
 */
typedef struct control_t {
  UrosMutex     lock;           /**< @brief Lock word.*/
  app_command_t command;
} control_t;

/** @brief Sequential stream on UART1 (SD1).*/
#define SS1                 ((BaseSequentialStream *)&SD1)

/** @brief Skips incoming data in handlers.*/
#define HANDLERS_INPUT_SKIP 0

/*===========================================================================*/
/* GLOBAL VARIABLES                                                          */
/*===========================================================================*/

extern control_t control;
// extern const UrosString rateparamname, sizeparamname;

/*===========================================================================*/
/* GLOBAL PROTOTYPES                                                         */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

void app_initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* _APP_H_ */
