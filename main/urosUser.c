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
 * @file    urosUser.c
 * @author  Andrea Zoppi <texzk@email.it>
 *
 * @brief   User-defined callback functions.
 */

/*===========================================================================*/
/* HEADER FILES                                                              */
/*===========================================================================*/

#include "urosHandlers.h"
#include "app.h"

#include <stdio.h>
#include <urosUser.h>
#include <urosRpcSlave.h>
#include <urosTcpRos.h>

/*===========================================================================*/
/* TYPES & MACROS                                                            */
/*===========================================================================*/

/** @brief   File where the Node configuration is stored. */
#define UROS_NODECONFIG_FILENAME    "urosNode.config"

/*===========================================================================*/
/* GLOBAL FUNCTIONS                                                          */
/*===========================================================================*/

/** @addtogroup user_funcs */
/** @{ */

/**
 * @brief   Loads node configuration.
 * @details Any previously allocated data is freed, then the configuration is
 *          loaded from a static non-volatile memory chunk.
 * @see     uros_lld_nodeconfig_load()
 *
 * @pre     The related @p UrosNode is initialized.
 *
 * @param[in,out] cfgp
 *          Pointer to the target configuration descriptor.
 */
void urosUserNodeConfigLoad(UrosNodeConfig *cfgp) {
  printf("reach urosUserNodeConfigLoad\n");
  urosAssert(cfgp != NULL);

  /* Clean any allocated variables.*/
  urosStringClean(&cfgp->nodeName);
  urosStringClean(&cfgp->xmlrpcUri);
  urosStringClean(&cfgp->tcprosUri);
  urosStringClean(&cfgp->masterUri);

  /* TODO: Load from EEPROM.*/
  urosNodeConfigLoadDefaults(cfgp);
}

/**
 * @brief   Saves the node configuration.
 * @details The node configuration is saved to a static non-volatile memory
 *          chunk.
 * @see     uros_lld_nodeconfig_save()
 *
 * @pre     The related @p UrosNode is initialized.
 *
 * @param[in] cfgp
 *          Pointer to the configuration descriptor to be saved.
 */
void urosUserNodeConfigSave(const UrosNodeConfig *cfgp) {

  /* TODO: Save to EEPROM.*/
  (void)cfgp;
}

/**
 * @brief   Shutdown callback function.
 * @details This callback function notifies the user that a @p shutdown()
 *          XMLRPC call was issued by the Master node, and has to be handled.
 *
 * @param[in] msgp
 *          Pointer to a string which explains the reason why it is asked to be
 *          shutdown.
 * @return
 *          Error code.
 */
uros_err_t urosUserShutdown(const UrosString *msgp) {
  printf("reach urosUserShutdown\n");

  static UrosNodeStatus *const stp = &urosNode.status;

  (void)msgp;
  (void)stp;

#if UROS_USE_ASSERT
  urosAssert(msgp != NULL);
  urosMutexLock(&stp->stateLock);
  urosAssert(stp->state == UROS_NODE_SHUTDOWN);
  urosMutexUnlock(&stp->stateLock);
#endif

  /* Send a dummy getPid() request, to unlock XMLRPC listener and pool.*/
  {
    UrosRpcResponse res;
    urosRpcResponseObjectInit(&res);
    urosRpcCallGetPid(
      &urosNode.config.xmlrpcAddr,
      &urosNode.config.nodeName,
      &res
    );
    urosRpcResponseClean(&res);
  }

  return UROS_OK;
}

/**
 * @brief   Registers static message types.
 * @details This callback function is called at boot time to initialize the
 *          set of message types recognized by the system.
 *
 * @pre     The global static message type set has not been initialized yet.
 */
void urosUserRegisterStaticTypes(void) {
  printf("reach urosUserRegisterStaticTypes\n");

  urosMsgTypesRegStaticTypes();
}

/**
 * @brief   Registers all the published topics to the Master node.
 * @note    Should be called at node initialization.
 *
 * @return  Error code.
 */
uros_err_t urosUserPublishTopics(void) {
  printf("reach urosUserPublishTopics\n");

  urosHandlersPublishTopics();
  return UROS_OK;
}

/**
 * @brief   Unregisters all the published topics to the Master node.
 * @note    Should be called at node shutdown.
 *
 * @return  Error code.
 */
uros_err_t urosUserUnpublishTopics(void) {
  printf("reach urosUserUnpublishTopics\n");

  urosHandlersUnpublishTopics();
  return UROS_OK;
}

/**
 * @brief   Registers all the subscribed topics to the Master node.
 * @note    Should be called at node initialization.
 *
 * @return  Error code.
 */
uros_err_t urosUserSubscribeTopics(void) {
  printf("reach urosUserSubscribeTopics\n");

  urosHandlersSubscribeTopics();
  return UROS_OK;
}

/**
 * @brief   Unregisters all the subscribed topics to the Master node.
 * @note    Should be called at node shutdown.
 *
 * @return  Error code.
 */
uros_err_t urosUserUnsubscribeTopics(void) {
  printf("reach urosUserUnsubscribeTopics\n");

  urosHandlersUnsubscribeTopics();
  return UROS_OK;
}

/**
 * @brief   Registers all the published services to the Master node.
 * @note    Should be called at node initialization.
 *
 * @return  Error code.
 */
uros_err_t urosUserPublishServices(void) {
  printf("reach urosUserPublishServices\n");

  urosHandlersPublishServices();
  return UROS_OK;
}

/**
 * @brief   Unregisters all the published services to the Master node.
 * @note    Should be called at node shutdown.
 *
 * @return  Error code.
 */
uros_err_t urosUserUnpublishServices(void) {
  printf("reach urosUserUnpublishServices\n");

  urosHandlersUnpublishServices();
  return UROS_OK;
}

/**
 * @brief   Registers all the subscribed parameters to the Master node.
 * @note    Should be called at node initialization.
 *
 * @return  Error code.
 */
uros_err_t urosUserSubscribeParams(void) {
  printf("reach urosUserSubscribeParams\n");

  static const UrosNodeConfig *const cfgp = &urosNode.config;

  UrosRpcParam paramval;
  UrosRpcResponse res;
  uros_err_t err; (void)err;

  urosRpcResponseObjectInit(&res);
  urosRpcParamObjectInit(&paramval, UROS_RPCP_INT);

  /* Output rate (packets/s).*/
  err = urosRpcCallHasParam(&cfgp->masterAddr, &cfgp->nodeName,
                            &rateparamname, &res);
  urosAssert(err == UROS_OK);
  if (!res.valuep->value.boolean) {
    urosRpcResponseClean(&res);
    paramval.value.int32 = 1;
    err = urosRpcCallSetParam(&cfgp->masterAddr, &cfgp->nodeName,
                              &rateparamname, &paramval, &res);
    urosAssert(err == UROS_OK);
  }
  urosRpcResponseClean(&res);
  urosNodeSubscribeParam(&rateparamname);

  /* Packet size (string length).*/
  err = urosRpcCallHasParam(&cfgp->masterAddr, &cfgp->nodeName,
                            &sizeparamname, &res);
  urosAssert(err == UROS_OK);
  if (!res.valuep->value.boolean) {
    urosRpcResponseClean(&res);
    paramval.value.int32 = 0;
    err = urosRpcCallSetParam(&cfgp->masterAddr, &cfgp->nodeName,
                              &sizeparamname, &paramval, &res);
    urosAssert(err == UROS_OK);
  }
  urosRpcResponseClean(&res);
  urosNodeSubscribeParam(&sizeparamname);

  urosRpcParamClean(&paramval, UROS_TRUE);
  return UROS_OK;
}

/**
 * @brief   Unregisters all the subscribed parameters to the Master node.
 * @note    Should be called at node shutdown.
 *
 * @return  Error code.
 */
uros_err_t urosUserUnsubscribeParams(void) {
  printf("reach urosUserUnsubscribeParams\n");

  urosNodeUnsubscribeParam(&rateparamname);
  urosNodeUnsubscribeParam(&sizeparamname);

  return UROS_OK;
}

/**
 * @brief   Updates a subscribed ROS parameter locally.
 * @details This callback function notifies the user that the value of a
 *          subscribed ROS parameter has changed.
 *
 * @param[in] keyp
 *          Pointer to the parameter name string.
 * @param[in] paramp
 *          Pointer to the parameter value.
 * @return
 *          Error code.
 */
uros_err_t urosUserParamUpdate(const UrosString *keyp,
                               const UrosRpcParam *paramp) {
  printf("reach urosUserParamUpdate\n");

  static const char hex[16] = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
  };

  (void)keyp;
  (void)paramp;

  urosAssert(urosStringNotEmpty(keyp));
  urosAssert(paramp != NULL);

  if (0 == urosStringCmp(keyp, &rateparamname)) {
    // urosAssert(paramp->class == UROS_RPCP_INT);
    benchmark.rate = (uint32_t)paramp->value.int32;
    return UROS_OK;
  }

  if (0 == urosStringCmp(keyp, &sizeparamname)) {
    size_t i;

    // urosAssert(paramp->class == UROS_RPCP_INT);
    urosMutexLock(&benchmark.lock);
    urosStringClean(&benchmark.payload);
    urosAssert(paramp->value.int32 >= 0);
    urosAssert(paramp->value.int32 < 100 * (1 << (20 - 3)));
    benchmark.payload.length = (uint32_t)paramp->value.int32;
    if (benchmark.payload.length > 0) {
      benchmark.payload.datap = urosArrayNew(NULL, benchmark.payload.length,
                                             char);
      for (i = 0; i < benchmark.payload.length; ++i) {
        benchmark.payload.datap[i] = hex[i & 0x0F];
      }
      urosAssert(benchmark.payload.datap != NULL);
    } else {
      benchmark.payload.datap = NULL;
    }
    urosMutexUnlock(&benchmark.lock);
    return UROS_OK;
  }

  /* Unknown parameter name.*/
  return UROS_ERR_BADPARAM;
}

/** @} */