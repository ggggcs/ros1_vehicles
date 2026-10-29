#pragma once
#include <string>
#include <functional>   //函数对象

#include "hv/hloop.h"
#include "hv/hssl.h"
#include "hv/hmutex.h"
#include "hv/hbase.h"
#include "hv/herr.h"

typedef std::function<void(void*, int)> TcpMessageCallback;

// @see mqtt/mqtt_client.h
typedef struct tcp_client_s {
    // connect: host:port
    char host[256];
    int  port;
    int  connect_timeout; // ms
    // reconnect
    reconn_setting_t* reconn_setting;
    // flags
    unsigned char   ssl: 1; // Read Only
    unsigned char   alloced_ssl_ctx: 1; // intern
    unsigned char   connected : 1;
    // privdata
    hloop_t*    loop;
    hio_t*      io;
    htimer_t*   reconn_timer;
    // SSL/TLS
    hssl_ctx_t  ssl_ctx;
    // thread-safe
    hmutex_t    mutex_;
    // ...
} tcp_client_t;

class TcpClient {
	public:

    static TcpClient *Instance(void);
    static void ReleaseInstance(void);

    void Start(std::string ip, int port, TcpMessageCallback call);

    void OnMessage(void* buf, int len);

    void Stop(void);

	private:

    static TcpClient *p_instance;
    
    tcp_client_t *cli_;
    TcpMessageCallback callback_;
};
