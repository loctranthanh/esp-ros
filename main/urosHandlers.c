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

/** @addtogroup tcpros_pubtopic_funcs */
/** @{ */

/*~~~ PUBLISHED TOPIC: /benchmark/output ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Topic <tt>/benchmark/output</tt> publisher */
/** @{ */

/**
 * @brief   TCPROS <tt>/benchmark/output</tt> published topic handler.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @return
 *          Error code.
 */
uros_err_t pub_tpc__benchmark__output(UrosTcpRosStatus *tcpstp) {

  uint32_t rate;

  /* Message allocation and initialization.*/
  UROS_TPC_INIT_H(msg__std_msgs__String);
  static char payload[] = "hello this is esp32\n";
  UrosString s_payload = {
    .datap = payload,
    .length = strlen(payload),
  };

  /* Published messages loop.*/
  while (!urosTcpRosStatusCheckExit(tcpstp)) {
    /* Assign the string chunk.*/
    // urosMutexLock(&benchmark.lock);
    // msgp->data = benchmark.payload;
    // rate = benchmark.rate;
    // urosMutexUnlock(&benchmark.lock);
    // memcpy(msgp->data.datap, payload, strlen(payload));
    // msgp->data.length = strlen(payload);
    msgp->data = s_payload;
    /* Send the message.*/
    UROS_MSG_SEND_LENGTH(msgp, msg__std_msgs__String);
    UROS_MSG_SEND_BODY(msgp, msg__std_msgs__String);
    printf("Send message to subscribers\n");
    // urosMutexLock(&benchmark.lock);
    // ++benchmark.outCount.numMsgs;
    // benchmark.outCount.numBytes += 2 * sizeof(uint32_t) + msgp->data.length;
    // ++benchmark.outCount.deltaMsgs;
    // benchmark.outCount.deltaBytes += 2 * sizeof(uint32_t) + msgp->data.length;
    // urosMutexUnlock(&benchmark.lock);

    /* No delay, to achieve the maximum throughput (beware: it may hang up).*/
    // if (rate > 0) {
    urosThreadSleepMsec(3000);
    // }
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
/* SUBSCRIBED TOPIC FUNCTIONS                                                */
/*===========================================================================*/

/** @addtogroup tcpros_subtopic_funcs */
/** @{ */

/*~~~ SUBSCRIBED TOPIC: /benchmark/input ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Topic <tt>/benchmark/input</tt> subscriber */
/** @{ */

/**
 * @brief   TCPROS <tt>/benchmark/input</tt> subscribed topic handler.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @return
 *          Error code.
 */
uros_err_t sub_tpc__benchmark__input(UrosTcpRosStatus *tcpstp) {
  printf("sub_tpc__benchmark__input\n");
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
    // msgp->data.datap[msglen] = '\0';
    printf("received: %.*s\n", msgp->data.length, msgp->data.datap);
    urosMutexLock(&benchmark.lock);
    ++benchmark.inCount.numMsgs;
    benchmark.inCount.numBytes += 2 * sizeof(uint32_t) + msgp->data.length;
    ++benchmark.inCount.deltaMsgs;
    benchmark.inCount.deltaBytes += 2 * sizeof(uint32_t) + msgp->data.length;
    urosMutexUnlock(&benchmark.lock);

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

/*~~~ SUBSCRIBED TOPIC: /benchmark/output ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Topic <tt>/benchmark/output</tt> subscriber */
/** @{ */

/**
 * @brief   TCPROS <tt>/benchmark/output</tt> subscribed topic handler.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @return
 *          Error code.
 */
uros_err_t sub_tpc__benchmark__output(UrosTcpRosStatus *tcpstp) {

  /* Message allocation and initialization.*/
  UROS_TPC_INIT_H(msg__std_msgs__String);

  /* Subscribed messages loop.*/
  while (!urosTcpRosStatusCheckExit(tcpstp)) {
    /* Receive the next message.*/
    UROS_MSG_RECV_LENGTH();
    UROS_MSG_RECV_BODY(msgp, msg__std_msgs__String);

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

  /* /benchmark/output */
  if (benchmark.hasOutPub) {
    urosNodePublishTopicSZ(
      "/esp_sensor",
      "std_msgs/String",
      (uros_proc_f)pub_tpc__benchmark__output,
      uros_nulltopicflags
    );
  }
}

/**
 * @brief   Unregisters all the published topics to the Master node.
 * @note    Should be called at node shutdown.
 */
void urosHandlersUnpublishTopics(void) {

  /* /benchmark/output */
  if (benchmark.hasOutPub) {
    urosNodeUnpublishTopicSZ(
      "/benchmark/output"
    );
  }
}

/**
 * @brief   Registers all the subscribed topics to the Master node.
 * @note    Should be called at node initialization.
 */
void urosHandlersSubscribeTopics(void) {
  printf("reach urosHandlersSubscribeTopics\n");

  /* /benchmark/input */
  if (benchmark.hasInSub) {
    urosNodeSubscribeTopicSZ(
      "/chatter",
      "std_msgs/String",
      (uros_proc_f)sub_tpc__benchmark__input,
      uros_nulltopicflags
    );
  }

  /* /benchmark/output */
  // if (benchmark.hasOutSub) {
  //   urosNodeSubscribeTopicSZ(
  //     "/benchmark/output",
  //     "std_msgs/String",
  //     (uros_proc_f)sub_tpc__benchmark__output,
  //     uros_nulltopicflags
  //   );
  // }
}

/**
 * @brief   Unregisters all the subscribed topics to the Master node.
 * @note    Should be called at node shutdown.
 */
void urosHandlersUnsubscribeTopics(void) {

  /* /benchmark/input */
  if (benchmark.hasInSub) {
    urosNodeUnsubscribeTopicSZ(
      "/chatter"
    );
  }

  /* /benchmark/output */
  // if (benchmark.hasOutSub) {
  //   urosNodeSubscribeTopicSZ(
  //     "/benchmark/output",
  //     "std_msgs/String",
  //     (uros_proc_f)sub_tpc__benchmark__output,
  //     uros_nulltopicflags
  //   );
  // }
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

