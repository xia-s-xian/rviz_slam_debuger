#include "yar_server_rpc.h"
#include "ros_interface.h"
#include "Helper.h"

#include <iostream>
#include <string.h>


void yar_map_handler(yar_request* req, yar_response* rsp, void* cokie)
{
    const yar_data* req_data = yar_request_get_parameters(req);
    uint pack_num = 0;
    if(yar_unpack_data_type(req_data,&pack_num) == YAR_DATA_ARRAY)
    {
        yar_unpack_iterator* pack_iterator = yar_unpack_iterator_init(req_data);
        do
        {
            const yar_data* map_data = yar_unpack_iterator_current(pack_iterator);
            uint kv_num = 0;
            if(yar_unpack_data_type(map_data,&kv_num) == YAR_DATA_MAP)
            {
                int sec;
                int nsec;
                char* frame_id = nullptr;
                float resolution;
                int width;
                int height;
                float position_x;
                float position_y;
                float position_z;
                float orientation_x;
                float orientation_y;
                float orientation_z;
                float orientation_w;
                char* data = nullptr;

                bool isError = false;

                yar_unpack_iterator* map_iterator = yar_unpack_iterator_init(map_data);
                do
                {
                    const yar_data* kv_data = yar_unpack_iterator_current(map_iterator);
                    char* key = new char[((msgpack_object_kv*)kv_data) ->key.via.str.size + 1];
                    memcpy(key,((msgpack_object_kv*)kv_data) ->key.via.str.ptr,((msgpack_object_kv*)kv_data) ->key.via.str.size);
                    key[((msgpack_object_kv*)kv_data)->key.via.str.size] = '\0';
                    if(strcmp(key,"sec") == 0)
                    {
                        sec = ((msgpack_object_kv*)kv_data)->val.via.i64;
                    }
                    else if(strcmp(key,"nsec") == 0)
                    {
                        nsec = ((msgpack_object_kv*)kv_data)->val.via.i64;
                    }
                    else if(strcmp(key,"frame_id") == 0)
                    {
                        frame_id = new char[ ((msgpack_object_kv*)kv_data)->val.via.str.size + 1 ];
                        memcpy(frame_id,((msgpack_object_kv*)kv_data)->val.via.str.ptr,((msgpack_object_kv*)kv_data)->val.via.str.size);
                        frame_id[((msgpack_object_kv*)kv_data)->val.via.str.size] = '\0';
                    }
                    else if(strcmp(key,"resolution") == 0)
                    {
                        resolution = ((msgpack_object_kv*)kv_data)->val.via.f64;
                    }
                    else if(strcmp(key,"width") == 0)
                    {
                        width = ((msgpack_object_kv*)kv_data)->val.via.i64;
                    }
                    else if(strcmp(key,"height") == 0)
                    {
                        height = ((msgpack_object_kv*)kv_data)->val.via.i64;
                    }
                    else if(strcmp(key,"px") == 0)
                    {
                        position_x = ((msgpack_object_kv*)kv_data)->val.via.f64;
                    }
                    else if(strcmp(key,"py") == 0)
                    {
                        position_y = ((msgpack_object_kv*)kv_data)->val.via.f64;
                    }
                    else if(strcmp(key,"pz") == 0)
                    {
                        position_z = ((msgpack_object_kv*)kv_data)->val.via.f64;
                    }
                    else if(strcmp(key,"ox") == 0)
                    {
                        orientation_x = ((msgpack_object_kv*)kv_data)->val.via.f64;
                    }
                    else if(strcmp(key,"oy") == 0)
                    {
                        orientation_y = ((msgpack_object_kv*)kv_data)->val.via.f64;
                    }
                    else if(strcmp(key,"oz") == 0)
                    {
                        orientation_z = ((msgpack_object_kv*)kv_data)->val.via.f64;
                    }
                    else if(strcmp(key,"ow") == 0)
                    {
                        orientation_w = ((msgpack_object_kv*)kv_data)->val.via.f64;
                    }
                    else if(strcmp(key,"data") == 0)
                    {
                        data = new char[ ((msgpack_object_kv*)kv_data)->val.via.str.size ];
                        memcpy(data,((msgpack_object_kv*)kv_data)->val.via.str.ptr,((msgpack_object_kv*)kv_data)->val.via.str.size);
                    }
                    else
                    {
                        std::cout << "[map] param error" << std::endl;
                        isError = true;
                        break;
                    }
                    yar_unpack_iterator_next(map_iterator);
                }while(yar_unpack_iterator_next(map_iterator));
                yar_unpack_iterator_free(map_iterator);

                if(!isError)
                {
                    nav_msgs::OccupancyGrid map;
                    Header* p = (Header*)&map;
                    p->nsec = nsec;
                    p->sec = sec;
                    p->seq = 0;
                    map.info.resolution = resolution;
                    map.info.width = width;
                    map.info.height = height;
                    map.info.origin.position.x = position_x;
                    map.info.origin.position.y = position_y;
                    map.info.origin.position.z = position_z;
                    map.info.origin.orientation.x = orientation_x;
                    map.info.origin.orientation.y = orientation_y;
                    map.info.origin.orientation.z = orientation_z;
                    map.info.origin.orientation.w = orientation_w;
                    map.data.resize(width * height);
                    for(int i = 0; i < width * height; i++)
                    {
                        map.data[i] = data[i];
                    }
                    map_pub.publish(map);
                }
                yar_packager* response;
                response = yar_pack_start_map(1);
                {
                    yar_pack_push_string(response,(char*)"error_code",strlen("error_code"));
                    if(isError)
                        yar_pack_push_long(response,1);
                    else
                        yar_pack_push_long(response,0);
                }
                yar_response_set_retval(rsp,response);
                yar_pack_free(response);
                delete frame_id;
                delete data;
            }
            else
            {
                std::cout << "[map] is not a map in array" << std::endl;
                break;
            }
        }while(yar_unpack_iterator_next(pack_iterator));
        yar_unpack_iterator_free(pack_iterator);
    }
    else
    {
        std::cout << "[map handler] is not a map" << std::endl;
    }
}
