#ifndef _ROS_test_Num_h
#define _ROS_test_Num_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace test
{

  class Num : public ros::Msg
  {
    public:
      typedef const char* _num_type;
      _num_type num;

    Num():
      num("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_num = strlen(this->num);
      varToArr(outbuffer + offset, length_num);
      offset += 4;
      memcpy(outbuffer + offset, this->num, length_num);
      offset += length_num;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_num;
      arrToVar(length_num, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_num; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_num-1]=0;
      this->num = (char *)(inbuffer + offset-1);
      offset += length_num;
     return offset;
    }

    virtual const char * getType() override { return "test/Num"; };
    virtual const char * getMD5() override { return "c8470232561560e383557c02344b87f7"; };

  };

}
#endif
