/**
 * @file include/kobuki_driver/packet_handler/payload_base.hpp
 *
 * @brief Base class for payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki_core/hydro-devel/kobuki_driver/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ROBOT_DATA_HPP_
#define ROBOT_DATA_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ecl/containers.hpp>
#include <stdint.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace packet_handler
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief
 * Provides base class for payloads.
 *
 */
class payloadBase
{
public:

  /**
   * this is simple magic to write the flag, when we get the packet from the host or
   * when we want to send the data
   */
  bool yes;

  /**
   * it indicates the type of derived packet. if packet type is dynamic, length of
   * packet can be changed during communication session. Ohterwise can not.
   */
  const bool is_dynamic;

  /**
   * it indicates length of data part of packet, except header and length field.
   * if packet is fixed type, this value should be matched with length field.
   * if packet is dynamic type, this value indicates minimal value of length field.
   */
  const unsigned char length;

  /*
   * construct and destruct
   */
  payloadBase(const bool is_dynamic_ = false, const unsigned char length_ = 0 )
    : yes(false)
    , is_dynamic(is_dynamic_)
    , length(length_)
  {};
  virtual ~payloadBase() {};

  /*
   * serialisation
   */
  virtual bool serialise(ecl::PushAndPop<unsigned char> & byteStream)=0;
  virtual bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)=0;

  // utilities
  // todo; let's put more useful converters here. Or we may use generic converters
protected:
  // below funciton should be replaced wiht converter
  template<typename T>
    void buildVariable(T & V, ecl::PushAndPop<unsigned char> & buffer)
    {
      if (buffer.size() < sizeof(T))
        return;
      V = static_cast<unsigned char>(buffer.pop_front());

      unsigned int size_value(sizeof(T));
      for (unsigned int i = 1; i < size_value; i++)
      {
        V |= ((static_cast<unsigned char>(buffer.pop_front())) << (8 * i));
      }
    }

  template<typename T>
    void buildBytes(const T & V, ecl::PushAndPop<unsigned char> & buffer)
    {
      unsigned int size_value(sizeof(T));
      for (unsigned int i = 0; i < size_value; i++)
      {
        buffer.push_back(static_cast<unsigned char>((V >> (i * 8)) & 0xff));
      }
    }
};

}

// namespace packet_handler

#endif /* ROBOT_DATA_HPP_ */
