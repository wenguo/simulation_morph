/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#ifndef COMMUNICATION_HH
#define COMMUNICATION_HH

#include <iostream>
#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>
namespace Morph
{
    class Robot;

    class Message
    {
        public:
            long timestamp;
            char * sender;
            char * receiver;
            char   type;
            uint8_t * data;
            int size;

            Message():sender(NULL),receiver(NULL),type('0'),data(NULL),size(0){}
            
            Message(const char * s, const char *r, const char t, const char *d, long ts)
            {
                sender = strdup(s);
                receiver = strdup(r);
                type = t;
                int len=strlen(d)+1;
                data = new uint8_t[len];
                memcpy(data, d, len);
                size = len;
                timestamp = ts;
            }

            Message(const char * s, const char *r, const char t, const uint8_t *d, int len, long ts)
            {
                sender = strdup(s);
                receiver = strdup(r);
                type = t;
                data = new uint8_t[len];
                memcpy(data, d, len);
                size = len;
                timestamp = ts;
            }

            Message(const Message &c)
            {
                sender = strdup(c.sender);
                receiver = strdup(c.receiver);
                type = c.type;
                data = new uint8_t[c.size];
                memcpy(data, c.data, c.size);
                size = c.size;
                timestamp = c.timestamp;
            }

            ~Message()
            {
                if (sender)
                {
                    free(sender);
                    sender=NULL;
                }

                if (receiver)
                {
                    free(receiver);
                    receiver=NULL;
                }

                if (data)
                {
                    free(data);
                    data=NULL;
                }
            }

            friend std::ostream & operator<<(std::ostream &os, const Message & m)
            {
                os<<m.sender<<" send message to "<<m.receiver<<"\n   type: "<<m.type <<"\t data:[ ";
                    for(int i=0;i<m.size;i++)
                        os<<(int)m.data[i]<<" ";
                os<<"]@"<<m.timestamp;
            }
    };


    class CommunicationBus
    {

        public:

            CommunicationBus():bus_id(0), expected_size(0) {}
            CommunicationBus(int id, int exp)
            {
                bus_id=id;
                expected_size=exp;
            }
            ~CommunicationBus()
            {
                CommunicationNodeList.clear();
            }
            void addCommunicationNode(Robot * node);
            void removeCommunicationNode(Robot * node);
            bool inNodeList(Robot *node);
            void printNodeList();
            int Size(){return CommunicationNodeList.size();};
            std::vector<Robot*> CommunicationNodeList;
            int bus_id;
            int expected_size;
    };

};
#endif
