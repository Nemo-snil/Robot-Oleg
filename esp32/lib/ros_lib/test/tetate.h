#ifndef _ROS_SERVICE_tetate_h
#define _ROS_SERVICE_tetate_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace test
{

static const char TETATE[] = "test/tetate";

  class tetateRequest : public ros::Msg
  {
    public:
      typedef int64_t _first_type;
      _first_type first;
      typedef int64_t _second_type;
      _second_type second;

    tetateRequest():
      first(0),
      second(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_first;
      u_first.real = this->first;
      *(outbuffer + offset + 0) = (u_first.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_first.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_first.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_first.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_first.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_first.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_first.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_first.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->first);
      union {
        int64_t real;
        uint64_t base;
      } u_second;
      u_second.real = this->second;
      *(outbuffer + offset + 0) = (u_second.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_second.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_second.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_second.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_second.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_second.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_second.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_second.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->second);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_first;
      u_first.base = 0;
      u_first.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_first.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_first.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_first.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_first.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_first.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_first.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_first.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->first = u_first.real;
      offset += sizeof(this->first);
      union {
        int64_t real;
        uint64_t base;
      } u_second;
      u_second.base = 0;
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->second = u_second.real;
      offset += sizeof(this->second);
     return offset;
    }

    virtual const char * getType() override { return TETATE; };
    virtual const char * getMD5() override { return "2c5dfe4179b944c72f370e8dc51ceb3c"; };

  };

  class tetateResponse : public ros::Msg
  {
    public:
      typedef int64_t _summa_type;
      _summa_type summa;

    tetateResponse():
      summa(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_summa;
      u_summa.real = this->summa;
      *(outbuffer + offset + 0) = (u_summa.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_summa.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_summa.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_summa.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_summa.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_summa.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_summa.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_summa.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->summa);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_summa;
      u_summa.base = 0;
      u_summa.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_summa.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_summa.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_summa.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_summa.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_summa.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_summa.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_summa.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->summa = u_summa.real;
      offset += sizeof(this->summa);
     return offset;
    }

    virtual const char * getType() override { return TETATE; };
    virtual const char * getMD5() override { return "07a7b3b51528b9e4d2a31a2319338cb8"; };

  };

  class tetate {
    public:
    typedef tetateRequest Request;
    typedef tetateResponse Response;
  };

}
#endif
