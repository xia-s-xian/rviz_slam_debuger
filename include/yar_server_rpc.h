#ifndef __YAR_SEERVER_RPC_H__
#define __YAR_SEERVER_RPC_H__

extern "C"
{
    #include <yar.h>
    #include <msgpack.h>
    void yar_map_handler(yar_request* req, yar_response* rsp, void* cokie);
}

#endif


