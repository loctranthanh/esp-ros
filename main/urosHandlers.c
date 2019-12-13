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
 * @file    urosHandlers.c
 * @author  Andrea Zoppi <texzk@email.it>
 *
 * @brief   TCPROS topic and service handlers.
 */

/*===========================================================================*/
/* HEADER FILES                                                              */
/*===========================================================================*/

#include "urosHandlers.h"
#include "app.h"

#include <urosNode.h>
#include <urosTcpRos.h>
#include <urosUser.h>

/*===========================================================================*/
/* PUBLISHED TOPIC FUNCTIONS                                                 */
/*===========================================================================*/

/**
 * @brief   TCPROS published topic handler.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @return
 *          Error code.
 */
uros_err_t pub_tpc__sensor__output(UrosTcpRosStatus *tcpstp) {

  uint32_t rate;

  /* Message allocation and initialization.*/
  UROS_TPC_INIT_H(msg__std_msgs__String);
  msgp->data.datap = malloc(1024);
  static char payload[] = "hello this is esp32\n";
  UrosString s_payload = {
    .datap = payload,
    .length = strlen(payload),
  };
  /* Published messages loop.*/
  while (!urosTcpRosStatusCheckExit(tcpstp)) {
    strncpy(msgp->data.datap, s_payload.datap, s_payload.length);
    msgp->data.length = s_payload.length;
    /* Send the message.*/
    UROS_MSG_SEND_LENGTH(msgp, msg__std_msgs__String);
    UROS_MSG_SEND_BODY(msgp, msg__std_msgs__String);
    printf("Send message to subscribers\n");
    urosThreadSleepMsec(3000);
  }
  free(msgp->data.datap);
  tcpstp->err = UROS_OK;

_finally:
  /* Message deinitialization and deallocation.*/
  UROS_TPC_UNINIT_H(msg__std_msgs__String);
  return tcpstp->err;
}

/** @} */

/** @} */

/*===========================================================================*/
/* SUBSCRIBED TOPIC FUNCTIONS                                                */
/*===========================================================================*/

/** @addtogroup tcpros_subtopic_funcs */
/** @{ */

/**
 * @brief   TCPROS subscribed topic handler.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @return
 *          Error code.
 */
uros_err_t sub_tpc__control__input(UrosTcpRosStatus *tcpstp) {
  /* Message allocation and initialization.*/
  UROS_TPC_INIT_H(msg__std_msgs__String);

  /* Subscribed messages loop.*/
  while (!urosTcpRosStatusCheckExit(tcpstp)) {
    /* Receive the next message.*/
    UROS_MSG_RECV_LENGTH();
#if HANDLERS_INPUT_SKIP
    msgp->data.length = msglen - sizeof(uint32_t);
    if (urosTcpRosSkip(tcpstp, msglen) != UROS_OK) { goto _finally; }
#else
    UROS_MSG_RECV_BODY(msgp, msg__std_msgs__String);
#endif
    printf("received: %.*s\n", msgp->data.length, msgp->data.datap);
    app_command_t received_command = STOP;
    if (!strncmp(msgp->data.datap, "UP", 2)) {
      received_command = FORWARD;
    } else if (!strncmp(msgp->data.datap, "DOWN", 4)) {
      received_command = BACKWARD;
    } else if (!strncmp(msgp->data.datap, "LEFT", 4)) {
      received_command = TURN_LEFT;
    } else if (!strncmp(msgp->data.datap, "RIGHT", 5)) {
      received_command = TURN_RIGHT;
    } else if (!strncmp(msgp->data.datap, "ROTATE_LEFT", 11)) {
      received_command = ROTATE_LEFT;
    } else if (!strncmp(msgp->data.datap, "ROTATE_RIGHT", 12)) {
      received_command = ROTATE_RIGHT;
    }
    urosMutexLock(&control.lock);
    control.command = received_command;
    urosMutexUnlock(&control.lock);

    /* Dispose the contents of the message.*/
    clean_msg__std_msgs__String(msgp);
  }
  tcpstp->err = UROS_OK;

_finally:
  /* Message deinitialization and deallocation.*/
  UROS_TPC_UNINIT_H(msg__std_msgs__String);
  return tcpstp->err;
}

/** @} */

/** @} */

/*===========================================================================*/
/* PUBLISHED SERVICE FUNCTIONS                                               */
/*===========================================================================*/

/** @addtogroup tcpros_pubservice_funcs */
/** @{ */

/* There are no published services.*/

/** @} */

/*===========================================================================*/
/* CALLED SERVICE FUNCTIONS                                                  */
/*===========================================================================*/

/** @addtogroup tcpros_callservice_funcs */
/** @{ */

/* There are no called services.*/

/** @} */

/*===========================================================================*/
/* GLOBAL FUNCTIONS                                                          */
/*===========================================================================*/

/** @addtogroup tcpros_funcs */
/** @{ */

/**
 * @brief   Registers all the published topics to the Master node.
 * @note    Should be called at node initialization.
 */
void urosHandlersPublishTopics(void) {
  printf("reach urosHandlersPublishTopics\n");

  urosNodePublishTopicSZ(
    "/esp_sensor",
    "std_msgs/String",
    (uros_proc_f)pub_tpc__sensor__output,
    uros_nulltopicflags
  );
}

/**
 * @brief   Unregisters all the published topics to the Master node.
 * @note    Should be called at node shutdown.
 */
void urosHandlersUnpublishTopics(void) {

}

/**
 * @brief   Registers all the subscribed topics to the Master node.
 * @note    Should be called at node initialization.
 */
void urosHandlersSubscribeTopics(void) {
  printf("reach urosHandlersSubscribeTopics\n");

  urosNodeSubscribeTopicSZ(
    "/controler",
    "std_msgs/String",
    (uros_proc_f)sub_tpc__control__input,
    uros_nulltopicflags
  );
}

/**
 * @brief   Unregisters all the subscribed topics to the Master node.
 * @note    Should be called at node shutdown.
 */
void urosHandlersUnsubscribeTopics(void) {

}

/**
 * @brief   Registers all the published services to the Master node.
 * @note    Should be called at node initialization.
 */
void urosHandlersPublishServices(void) {

  /* No services to publish.*/
}

/**
 * @brief   Unregisters all the published services to the Master node.
 * @note    Should be called at node shutdown.
 */
void urosHandlersUnpublishServices(void) {

  /* No services to unpublish.*/
}

/** @} */

