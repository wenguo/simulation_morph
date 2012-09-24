/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#ifndef IPC_HH
#define IPC_HH
#include "lolmsg.h"
#include "bytequeue.h"

#define IPCLOLBUFFERSIZE 320*240*3  
#define IPCTXBUFFERSIZE  320*240*3
#define IPCBLOCKSIZE  1024

namespace IPC{

class IPC
{
    typedef void (*Callback)(const LolMessage *, void *);

    public:
        IPC(int _ipc_txbuffer_size = IPCTXBUFFERSIZE, int _ipc_lolbuffer_size=IPCLOLBUFFERSIZE, int _ipc_block_size = IPCBLOCKSIZE);
        ~IPC(){};

        bool Start(const char *host,int port, bool server);
	bool Exit();
        bool SendData(const uint8_t type, uint8_t *data, int len);
        inline void SetCallback(Callback c, void * u) {callback = c; user_data = u;}
        int id;

    private:
        static int index;
        static void * Monitoring(void *ptr);
        static void * Receiving(void *ptr);
        static void * Transmiting(void *ptr);
        bool StartServer(int port);
        bool ConnectToServer(const char * host, int port);
        bool ListenForConnection(int sockfd);
        
        Callback callback;
        void * user_data;

        int sockfd;
        int clientsockfd;
        bool server;
        int port;
        char* host;
        bool connected;
        uint8_t *txbuffer;//[IPCTXBUFFERSIZE];
        ByteQueue txq;
        LolParseContext parseContext;
        pthread_t monitor_thread;
        pthread_t receiving_thread;
        pthread_t transmiting_thread;
        pthread_mutex_t mutex_txq;

        int ipc_block_size;
        int ipc_lolbuffer_size;
        int ipc_txbuffer_size;

};

}//end of namespace
#endif
