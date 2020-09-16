/**
 * @file include/kobuki_driver/packets/firmware.hpp
 *
 * @brief Firmware version request packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki_core/hydro-devel/kobuki_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_FW_DATA_HPP__
#define KOBUKI_FW_DATA_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include <vector>

#include "../packet_handler/payload_base.hpp"
#include "../packet_handler/payload_headers.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/

class Firmware : public packet_handler::payloadBase
{
public:
  static const uint32_t COMPATIBLE_MAJOR_VERSION;  // protocol communication will fail
  static const std::vector<uint32_t> RECOMMENDED_VERSIONS;  // advisory minor.patch recommendations

  Firmware() : packet_handler::payloadBase(true, 2) {};

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    unsigned char length = 4;
    buildBytes(Header::Firmware, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.version, byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream);

  bool constrain()
  {
    return true;
  }

  void showMe()
  {
  }

  const uint32_t& version() const { return data.version; }
  int majorVersion() const { return VersionInfo::majorVersion(data.version); }
  int minorVersion() const { return VersionInfo::minorVersion(data.version); }
  int patchVersion() const { return VersionInfo::patchVersion(data.version); }

  int checkMajorVersion() const;
  int checkRecommendedVersion() const;

private:
  struct Data {
    uint32_t version;
  };
  Data data;
};

} // namespace kobuki

#endif /* KOBUKI_FW_DATA_HPP__ */

