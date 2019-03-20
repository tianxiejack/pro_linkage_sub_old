/*
 * Ipc.hpp
 *
 *  Created on: May 04, 2018
 *      Author: ubuntu
 */

#ifndef IPC_HPP_
#define IPC_HPP_


void Ipc_pthread_start(void);
void Ipc_pthread_stop(void);
extern void MSGAPI_msgsend(int cmdID);
extern int send_msg(SENDST *RS422);


#endif /* IPC_HPP_ */

