/*                                                                             
 * copyright (c) 2013, Yujin Robot.
 * all rights reserved.
 *
 * redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * neither the name of yujin robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * this software is provided by the copyright holders and contributors "as is"
 * and any express or implied warranties, including, but not limited to, the
 * implied warranties of merchantability and fitness for a particular purpose
 * are disclaimed. in no event shall the copyright owner or contributors be
 * liable for any direct, indirect, incidental, special, exemplary, or
 * consequential damages (including, but not limited to, procurement of
 * substitute goods or services; loss of use, data, or profits; or business
 * interruption) however caused and on any theory of liability, whether in
 * contract, strict liability, or tort (including negligence or otherwise)
 * arising in any way out of the use of this software, even if advised of the
 * possibility of such damage.
 */
/**
 * @file /kobuki_driver/src/driver/dock_drive.cpp
 *
 **/

/*****************************************************************************
** includes
*****************************************************************************/

#include "kobuki_dock_drive/dock_drive.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

  void DockDrive::nearCenter(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const ecl::Pose2D<double>& pose_update)
  {
    unsigned int right = signal_filt[0];
    unsigned int mid  = signal_filt[1];
    unsigned int left = signal_filt[2];
    // if mid ir sees center signal
    if(mid & DockStationIRState::NEAR_CENTER) setVel(0.05, 0.0);
    // if left ir sees center signal 
    if(left & DockStationIRState::NEAR_CENTER) setVel(0.0, 0.1);
    // if right ir sees center signal
    if(right & DockStationIRState::NEAR_CENTER) setVel(0.0, -0.1);
  }

  void DockDrive::farCenter(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const ecl::Pose2D<double>& pose_update)
  {
    unsigned int right = signal_filt[0];
    unsigned int mid  = signal_filt[1];
    unsigned int left = signal_filt[2];
    // if mid ir sees center signal
    if(mid & DockStationIRState::FAR_CENTER) setVel(0.1, 0.0);
    // if left ir sees center signal 
    if(left & DockStationIRState::FAR_CENTER) setVel(0.0, 0.3);
    // if right ir sees center signal
    if(right & DockStationIRState::FAR_CENTER) setVel(0.0, -0.3);
  }

  void DockDrive::nearLeft(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const ecl::Pose2D<double>& pose_update)
  {
    unsigned int right = signal_filt[0];
    unsigned int mid  = signal_filt[1];
    unsigned int left = signal_filt[2];
    // if mid ir sees center signal
    if(mid & DockStationIRState::NEAR_LEFT) setVel(0.0, 0.3);
    // if left ir sees center signal 
    if(left & DockStationIRState::NEAR_LEFT) setVel(0.0, 0.3);
    // if right ir sees center signal
    if(right & DockStationIRState::NEAR_LEFT) setVel(0.05, 0);
  }

  void DockDrive::farLeft(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const ecl::Pose2D<double>& pose_update)
  {
    unsigned int right = signal_filt[0];
    unsigned int mid  = signal_filt[1];
    unsigned int left = signal_filt[2];
    // if mid ir sees center signal
    if(mid & DockStationIRState::FAR_LEFT) setVel(0.0, 0.3);
    // if left ir sees center signal 
    if(left & DockStationIRState::FAR_LEFT) setVel(0.0, 0.3);
    // if right ir sees center signal
    if(right & DockStationIRState::FAR_LEFT) setVel(0.1, 0.0);
  }

  void DockDrive::nearRight(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const ecl::Pose2D<double>& pose_update)
  {
    unsigned int right = signal_filt[0];
    unsigned int mid  = signal_filt[1];
    unsigned int left = signal_filt[2];
    // if mid ir sees center signal
    if(mid & DockStationIRState::NEAR_RIGHT) setVel(0.0, -0.3);
    // if left ir sees center signal 
    if(left & DockStationIRState::NEAR_RIGHT) setVel(0.05, 0.0);
    // if right ir sees center signal
    if(right & DockStationIRState::NEAR_RIGHT) setVel(0.0, -0.5);
  }

  void DockDrive::farRight(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const ecl::Pose2D<double>& pose_update)
  {
    unsigned int right = signal_filt[0];
    unsigned int mid  = signal_filt[1];
    unsigned int left = signal_filt[2];
    // if mid ir sees center signal
    if(mid & DockStationIRState::FAR_RIGHT) setVel(0.0, -0.3);
    // if left ir sees center signal 
    if(left & DockStationIRState::FAR_RIGHT) setVel(0.05, 0.0);
    // if right ir sees center signal
    if(right & DockStationIRState::FAR_RIGHT) setVel(0.0, -0.5);
  }

} 
