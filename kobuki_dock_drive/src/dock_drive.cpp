/*
 * copyright (c) 2013, yujin robot.
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
** defines
*****************************************************************************/

#define sign(x) (x>0?+1:x<0?-1:0)
#define stringfy(x) #x
#define setState(x) {state=x;}
#define setStateVel(x,v,w) {setState(x);setVel(v,w);}

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/
DockDrive::DockDrive() :
  is_enabled(false), can_run(false) 
  , state(RobotDockingState::IDLE), state_str("IDLE")
  , vx(0.0), wz(0.0)
  , bump_remainder(0)
  , dock_stabilizer(0)
  , dock_detector(0)
  , rotated(0.0)
  , min_abs_v(0.01)
  , min_abs_w(0.1)
  , signal_window(20)
  , ROBOT_STATE_STR(13)
{

  ROBOT_STATE_STR[0] = "IDLE";
  ROBOT_STATE_STR[1] = "DONE";
  ROBOT_STATE_STR[2] = "DOCKED_IN";
  ROBOT_STATE_STR[3] = "BUMPED_DOCK";
  ROBOT_STATE_STR[4] = "BUMPED";
  ROBOT_STATE_STR[5] = "SCAN";
  ROBOT_STATE_STR[6] = "FIND_STREAM";
  ROBOT_STATE_STR[7] = "GET_STREAM";
  ROBOT_STATE_STR[8] = "ALIGNED";
  ROBOT_STATE_STR[9] = "ALIGNED_FAR";
  ROBOT_STATE_STR[10] = "ALIGNED_NEAR";
  ROBOT_STATE_STR[11] = "UNKNOWN";
  ROBOT_STATE_STR[12] = "LOST";
  /*
    "DONE",
    "DOCKED_IN",
    "BUMPED_DOCK",
    "BUMPED",
    "SCAN",
    "FIND_STREAM",
    "GET_STREAM",
    "ALIGNED",
    "ALIGNED_FAR",
    "ALIGNED_NEAR",
    "UNKNOWN",
    "LOST"};
*/

}

DockDrive::~DockDrive(){;}

void DockDrive::setVel(double v, double w)
{
  vx = sign(v) * std::max(std::abs(v), min_abs_v);
  wz = sign(w) * std::max(std::abs(w), min_abs_w);
}

void DockDrive::modeShift(const std::string& mode)
{
  if (mode == "enable")  { is_enabled = true;  can_run = true; state = RobotDockingState::IDLE;}
  if (mode == "disable") { is_enabled = false; can_run = false; }
  if (mode == "run")  can_run = true;
  if (mode == "stop") can_run = false;
}


/**
 * @brief Updates the odometry from firmware stamps and encoders.
 *
 * Really horrible - could do with an overhaul.
 *
 * @param dock_ir signal
 * @param bumper sensor
 * @param charger sensor
 * @param current pose
 */
void DockDrive::update(const std::vector<unsigned char> &signal
                , const unsigned char &bumper
                , const unsigned char &charger
                , const ecl::Pose2D<double>& pose) {

  ecl::Pose2D<double> pose_update;
  std::vector<unsigned char> signal_filt(signal.size(), 0);
  std::string debug_str;

  // process bumper and charger event first
  // docking algorithm terminates here 
  if(bumper || charger) {
    processBumpChargeEvent(bumper, charger);
  }
  else {
    /*************************
     * pre processing
     *************************/
    computePoseUpdate(pose_update, pose);
    filterIRSensor(signal_filt, signal);
    updateVelocity(signal_filt, pose_update, debug_str);
  }

  velocityCommands(vx, wz);

  // for easy debugging
  generateDebugMessage(signal_filt, bumper, charger, pose_update, debug_str);

  return;
}

/**
 * @brief compute pose update from previouse pose and current pose
 *
 * @param pose update. this variable get filled after this function 
 * @param pose - current pose
 **/
void DockDrive::computePoseUpdate(ecl::Pose2D<double>& pose_update, const ecl::Pose2D<double>& pose)
{
  double dx = pose.x() - pose_priv.x();
  double dy = pose.y() - pose_priv.y();
  pose_update.x( std::sqrt( dx*dx + dy*dy ) );
  pose_update.heading( pose.heading() - pose_priv.heading() );
  //std::cout << pose_diff << "=" << pose << "-" << pose_priv << " | " << pose_update << std::endl;
  pose_priv = pose;

}


/**
 * @breif pushing into signal into signal window. and go through the signal window to find what has detected
 *
 * @param signal_filt - this get filled out after the function. 
 * @param signal - the raw data from robot
 **/

void DockDrive::filterIRSensor(std::vector<unsigned char>& signal_filt,const std::vector<unsigned char> &signal)
{
  //dock_ir signals filtering
  past_signals.push_back(signal);
  while (past_signals.size() > signal_window) {
    past_signals.erase( past_signals.begin(), past_signals.begin() + past_signals.size() - signal_window);
  }

  for ( unsigned int i = 0; i < past_signals.size(); i++) {
    if (signal_filt.size() != past_signals[i].size()) 
      continue;
    for (unsigned int j = 0; j < signal_filt.size(); j++)
      signal_filt[j] |= past_signals[i][j];
  }
}


void DockDrive::velocityCommands(const double &vx_, const double &wz_) {
  // vx: in m/s
  // wz: in rad/s
  vx = vx_;
  wz = wz_;
}

void DockDrive::processBumpChargeEvent(const unsigned char& bumper, const unsigned char& charger) {
  if(charger && bumper) {
    setStateVel(RobotDockingState::BUMPED_DOCK, -0.01, 0.0); 
  }
  else if(charger) {
    if(dock_stabilizer == 0) {
      setStateVel(RobotDockingState::DOCKED_IN, 0.0, 0.0); 
    }
    else if(dock_stabilizer > 20) {
      dock_stabilizer = 0;
      is_enabled = false;
      can_run = false;
      setStateVel(RobotDockingState::IDLE, 0.0, 0.0);
    }
    else {
      dock_stabilizer++;
      setStateVel(RobotDockingState::DOCKED_IN, 0.0, 0.0); 
    }
  }
  else if(bumper) {
    setStateVel(RobotDockingState::BUMPED, -0.05, 0.0);
  }
}

void DockDrive::updateVelocity(const std::vector<unsigned char>& signal_filt, const ecl::Pose2D<double>& pose_update, std::string& debug_str)
{
  std::ostringstream oss;
  RobotDockingState::State current_state, new_state;
  double new_vx = 0.0;
  double new_vy = 0.0;

  // determine the current state based on ir and the previous state
  // common transition. idle -> scan -> find_stream -> get_stream -> scan -> aligned_far -> aligned_near -> docked_in -> done

  current_state = new_state = state;
  switch((unsigned int)current_state) {
    case RobotDockingState::IDLE:
      idle(new_state, new_vx, new_vy);
      break;
    case RobotDockingState::SCAN:
      scan(new_state, new_vx, new_vy, signal_filt, pose_update, debug_str);
      break;
    case RobotDockingState::FIND_STREAM:
      find_stream(new_state, new_vx, new_vy, signal_filt);
      break;
    case RobotDockingState::GET_STREAM:
      get_stream(new_state, new_vx, new_vy, signal_filt);
      break;
    case RobotDockingState::ALIGNED:
    case RobotDockingState::ALIGNED_FAR:
    case RobotDockingState::ALIGNED_NEAR:
      aligned(new_state, new_vx, new_vy, signal_filt, debug_str);
      break;
    default:
      oss << "Wrong state : " << current_state;
      debug_str = oss.str();
      break;
  }

  setStateVel(new_state, new_vx, new_vy);
  state_str = ROBOT_STATE_STR[new_state];
  /*
  std::ostringstream os;
  os << new_state;
  state_str = os.str(); 
  */
}

/*************************
 * @breif Check if any ir sees the given state signal from dock station 
 * 
 * @param filtered signal 
 * @param dock ir state
 *
 * @ret true or false
 *************************/
bool DockDrive::validateSignal(const std::vector<unsigned char>& signal_filt, const unsigned int state)
{
  unsigned int i;
  for(i = 0; i < signal_filt.size(); i++)
  {
    if(signal_filt[i] & state)
      return true;
  }
  return false;
}

/*************************
 * @breif processing. algorithms; transforma to velocity command
 *
 * @param dock_ir signal
 * @param bumper sensor
 * @param charger sensor
 * @param pose_update
 *
 *************************/
/*
void DockDrive::updateVelocity(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const ecl::Pose2D<double>& pose_update, std::string& debug_str)
{

  //std::string debug_str = "";
  do {  // a kind of hack
    if ( state==RobotDockingState::DONE ) setState(RobotDockingState::IDLE); // when this function is called after final state 'DONE'.
    if ( state==RobotDockingState::DOCKED_IN ) {
      if ( dock_stabilizer++ > 20 ) {
        is_enabled = false;
        can_run = false;
        setStateVel(RobotDockingState::DONE, 0.0, 0.0); break;
      }
      setStateVel(RobotDockingState::DOCKED_IN, 0.0, 0.0); break;
    }
    if ( bump_remainder > 0 ) {
      bump_remainder--;
      if ( charger ) { setStateVel(RobotDockingState::DOCKED_IN, 0.0, 0.0); break; } // when bumper signal is received early than charger(dock) signal.
      else {           setStateVel(RobotDockingState::BUMPED_DOCK, -0.01, 0.0); break; }
    } else if (state == RobotDockingState::BUMPED) {
      setState(RobotDockingState::IDLE); //should I remember and recall previous state?
      debug_str="how dare!!";
    }
    if ( bumper || charger ) {
      if( bumper && charger ) {
        bump_remainder = 0;
        setStateVel(RobotDockingState::BUMPED_DOCK, -0.01, 0.0); break;
      }
      if ( bumper ) {
        bump_remainder = 50;
        setStateVel(RobotDockingState::BUMPED, -0.05, 0.0); break;
      }
      if ( charger ) { // already docked in
        dock_stabilizer = 0;
        setStateVel(RobotDockingState::DOCKED_IN, 0.0, 0.0); break;
      }
    } else {
      if ( state==RobotDockingState::IDLE ) {
        dock_detector = 0;
        rotated = 0.0;
        setStateVel(RobotDockingState::SCAN, 0.00, 0.66); break;
      }
      if ( state==RobotDockingState::SCAN ) {
        rotated += pose_update.heading()/(2.0*M_PI);
        std::ostringstream oss;
        oss << "rotated: " << std::fixed << std::setprecision(2) << std::setw(4) << rotated;
        debug_str = oss.str();
        if( std::abs(rotated) > 1.6 ) {
          setStateVel(RobotDockingState::FIND_STREAM, 0.0, 0.0); break;
        }
        if (  signal_filt[1]&(DockStationIRState::FAR_LEFT  + DockStationIRState::NEAR_LEFT )) dock_detector--;
        if (  signal_filt[1]&(DockStationIRState::FAR_RIGHT + DockStationIRState::NEAR_RIGHT)) dock_detector++;
        if ( (signal_filt[1]&DockStationIRState::FAR_CENTER) || (signal_filt[1]&DockStationIRState::NEAR_CENTER) ) {
          setStateVel(RobotDockingState::ALIGNED, 0.05, 0.00); break;
        } else if ( signal_filt[1] ) {
          setStateVel(RobotDockingState::SCAN, 0.00, 0.10); break;
        } else {
          setStateVel(RobotDockingState::SCAN, 0.00, 0.66); break;
        }

      } else if (state==RobotDockingState::ALIGNED || state==RobotDockingState::ALIGNED_FAR || state==RobotDockingState::ALIGNED_NEAR) {
        if ( signal_filt[1] ) {
          if ( signal_filt[1]&DockStationIRState::NEAR )
          {
            if ( ((signal_filt[1]&DockStationIRState::NEAR) == DockStationIRState::NEAR_CENTER) || ((signal_filt[1]&DockStationIRState::NEAR) == DockStationIRState::NEAR) ) { setStateVel(RobotDockingState::ALIGNED_NEAR, 0.05,  0.0); debug_str = "AlignedNearCenter"; break; }
            if (   signal_filt[1]&DockStationIRState::NEAR_LEFT  ) {                                               setStateVel(RobotDockingState::ALIGNED_NEAR, 0.05,  0.1); debug_str = "AlignedNearLeft"  ; break; }
            if (   signal_filt[1]&DockStationIRState::NEAR_RIGHT ) {                                               setStateVel(RobotDockingState::ALIGNED_NEAR, 0.05, -0.1); debug_str = "AlignedNearRight" ; break; }
          }
          if ( signal_filt[1]&DockStationIRState::FAR )
          {
            if ( ((signal_filt[1]&DockStationIRState::FAR) == DockStationIRState::FAR_CENTER) || ((signal_filt[1]&DockStationIRState::FAR) == DockStationIRState::FAR) ) { setStateVel(RobotDockingState::ALIGNED_FAR, 0.1,  0.0); debug_str = "AlignedFarCenter"; break; }
            if (   signal_filt[1]&DockStationIRState::FAR_LEFT  ) {                                            setStateVel(RobotDockingState::ALIGNED_FAR, 0.1,  0.3); debug_str = "AlignedFarLeft"  ; break; }
            if (   signal_filt[1]&DockStationIRState::FAR_RIGHT ) {                                            setStateVel(RobotDockingState::ALIGNED_FAR, 0.1, -0.3); debug_str = "AlignedFarRight" ; break; }
          }
          dock_detector = 0;
          rotated = 0.0;
          setStateVel(RobotDockingState::SCAN, 0.00, 0.66); break;
        } else {
          debug_str = "lost signals";
          setStateVel(RobotDockingState::LOST, 0.00, 0.00); break;
        }
      } else if (state==RobotDockingState::FIND_STREAM) {
        if (dock_detector > 0 ) { // robot is placed in right side of docking station
          //turn  right , negative direction til get right signal from left sensor
          if (signal_filt[2]&(DockStationIRState::FAR_RIGHT+DockStationIRState::NEAR_RIGHT)) {
            setStateVel(RobotDockingState::GET_STREAM, 0.05, 0.0); break;
          } else {
            setStateVel(RobotDockingState::FIND_STREAM, 0.0, -0.33); break;
          }
        } else if (dock_detector < 0 ) { // robot is placed in left side of docking station
          //turn left, positive direction till get left signal from right sensor
          if (signal_filt[0]&(DockStationIRState::FAR_LEFT+DockStationIRState::NEAR_LEFT)) {
            setStateVel(RobotDockingState::GET_STREAM, 0.05, 0.0); break;
          } else {
            setStateVel(RobotDockingState::FIND_STREAM, 0.0, 0.33); break;
          }
        }
      } else if (state==RobotDockingState::GET_STREAM) {
        if (dock_detector > 0) { //robot is placed in right side of docking station
          if (signal_filt[2]&(DockStationIRState::FAR_LEFT+DockStationIRState::NEAR_LEFT)) {
            dock_detector = 0;
            rotated = 0.0;
            setStateVel(RobotDockingState::SCAN, 0.0, 0.10); break;
          } else {
            setStateVel(RobotDockingState::GET_STREAM, 0.05, 0.0); break;
          }
        } else if (dock_detector < 0) { // robot is placed in left side of docking station
          if (signal_filt[0]&(DockStationIRState::FAR_RIGHT+DockStationIRState::NEAR_RIGHT)) {
            dock_detector = 0;
            rotated = 0.0;
            setStateVel(RobotDockingState::SCAN, 0.0, 0.10); break;
          } else {
            setStateVel(RobotDockingState::GET_STREAM, 0.05, 0.0); break;
          }
        }
      } else {
        dock_detector = 0;
        rotated = 0.0;
        setStateVel(RobotDockingState::SCAN, 0.00, 0.66); break;
      }
    }
    setStateVel(RobotDockingState::UNKNOWN, 0.00, 0.00); break;
  } while(0);
}*/
} // kobuki namespace
