/*
 * copyright (c) 2012, yujin robot.
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
#define setState(x) {state=x;state_str=stringfy(x);}
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
{
}

DockDrive::~DockDrive(){;}

void DockDrive::setVel(double v, double w)
{
  vx = sign(v) * std::max(std::abs(v), min_abs_v);
  wz = sign(w) * std::max(std::abs(w), min_abs_w);
}

void DockDrive::modeShift(const std::string& mode)
{
  if (mode == "enable")  { is_enabled = true;  can_run = true; }
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

  /*************************
   * pre processing
   *************************/
  computePoseUpdate(pose_update, pose);
  filterIRSensor(signal_filt, signal);

  // 1. determine the location of robot
  state = determineRobotLocation(signal_filt, charger);

  // 2. decide the robot's behavior
  // 3. publish velocity
  // state transition 
  updateVelocity(signal_filt, bumper, charger, pose_update, debug_str);

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

std::string DockDrive::binary(unsigned char number) const {
  std::string ret;
  for( unsigned int i=0;i<6; i++){
    if (number&1) ret = "1" + ret;
    else          ret = "0" + ret;
    number = number >> 1;
  }
  return ret;
}


void DockDrive::generateDebugMessage(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const ecl::Pose2D<double>& pose_update, const std::string& debug_str)
{
  /*************************
   * debug prints
   *************************/
  std::ostringstream debug_stream;
  // pose_update and pose_update_rates for debugging
  debug_stream << std::fixed << std::setprecision(4)
    << "[x: "    << std::setw(7) << pose_update.x()
    << ", y: "  << std::setw(7) << pose_update.y()
    << ", th: " << std::setw(7) << pose_update.heading()
    << "]";

  //dock_ir signal
  /*
  debug_stream 
    << "[l: "  << binary(signal_filt[2])
    << ", c: " << binary(signal_filt[1])
    << ", r: " << binary(signal_filt[0])
    << "]";
  */
  std::string far_signal  = "[F: "; //far field
  std::string near_signal = "[N: "; //near field
  for (unsigned int i=0; i<3; i++) {
    if (signal_filt[2-i]&DockStationIRState::FAR_LEFT   ) far_signal  += "L"; else far_signal  += "-";
    if (signal_filt[2-i]&DockStationIRState::FAR_CENTER ) far_signal  += "C"; else far_signal  += "-";
    if (signal_filt[2-i]&DockStationIRState::FAR_RIGHT  ) far_signal  += "R"; else far_signal  += "-";
    if (signal_filt[2-i]&DockStationIRState::NEAR_LEFT  ) near_signal += "L"; else near_signal += "-";
    if (signal_filt[2-i]&DockStationIRState::NEAR_CENTER) near_signal += "C"; else near_signal += "-";
    if (signal_filt[2-i]&DockStationIRState::NEAR_RIGHT ) near_signal += "R"; else near_signal += "-";
    far_signal  += " ";
    near_signal += " ";
  }
  far_signal  += "]";
  near_signal += "]";
  debug_stream << far_signal << near_signal;

  //bumper
  {
  std::string out = "[B: ";
  if (bumper&4) out += "L"; else out += "-";
  if (bumper&2) out += "C"; else out += "-";
  if (bumper&1) out += "R"; else out += "-";
  out += "]";
  debug_stream << out;
  }

  //charger
  {
  std::ostringstream oss;
  oss << "[C:" << std::setw(2) << (unsigned int)charger;
  oss << "(";
  if (charger) oss << "ON"; else oss << "  ";
  oss << ")]";
  debug_stream << oss.str();
  }

  //debug_stream << std::fixed << std::setprecision(4)
  debug_stream << "[vx: " << std::setw(7) << vx << ", wz: " << std::setw(7) << wz << "]";
  debug_stream << "[S: " << state << "]";
  debug_stream << "[" << debug_str << "]";
  //debug_stream << std::endl;
  debug_output = debug_stream.str();

  //std::cout << debug_output << std::endl;;
}

void DockDrive::updateVelocity(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const ecl::Pose2D<double>& pose_update, std::string& debug_str)
{
  std::ostringstream oss;

  // determine the current state based on ir and the previous state
  switch((unsigned int)state) {
    case RobotDockingState::IN_DOCK:
      oss << "In dock. Mark it as done";
      state = RobotDockingState::DONE;
      break;
    case RobotDockingState::NEAR_CENTER:
      oss << "Near Center";
      nearCenter(signal_filt, bumper, charger, pose_update);
      break;
    case RobotDockingState::FAR_CENTER:
      oss << "Far Center";
      farCenter(signal_filt, bumper, charger, pose_update);
      break;
    case RobotDockingState::NEAR_LEFT:
      oss << "Near Left";
      nearLeft(signal_filt, bumper, charger, pose_update);
      break;
    case RobotDockingState::FAR_LEFT:
      oss << "Far Left";
      farLeft(signal_filt, bumper, charger, pose_update);
      break;
    case RobotDockingState::NEAR_RIGHT:
      oss << "Near Right";
      nearRight(signal_filt, bumper, charger, pose_update);
      break;
    case RobotDockingState::FAR_RIGHT:
      oss << "Far Right";
      farRight(signal_filt, bumper, charger, pose_update);
      break;
    default:
      oss << "Wrong state : " << state;
      break;
  }
  debug_str = oss.str();
}

RobotDockingState::State DockDrive::determineRobotLocation(const std::vector<unsigned char>& signal_filt, const unsigned char& charger)
{
  // IR 
  /*
     R - signal_filt[0] 
     C - signal_filt[1]
     L - signal_filt[2]

     values - DockStationIRState   

     Determine among
      NEAR_LEFT, NEAR_CENTER, NEAR_RIGHT
      FAR_LEFT, FAR_CENTER, FAR_RIGHT
   */                                      
  DockStationIRState::State current_state;

  unsigned int robot_left = signal_filt[2];
  unsigned int robot_mid = signal_filt[1];
  unsigned int robot_right = signal_filt[0];

  unsigned int dock_signal_array[6][2] = 
                { { DockStationIRState::NEAR_CENTER, RobotDockingState::NEAR_CENTER},
                  { DockStationIRState::FAR_CENTER, RobotDockingState::FAR_CENTER},
                  { DockStationIRState::NEAR_LEFT, RobotDockingState::NEAR_LEFT},
                  { DockStationIRState::FAR_LEFT, RobotDockingState::FAR_LEFT},
                  { DockStationIRState::NEAR_RIGHT, RobotDockingState::NEAR_RIGHT},
                  { DockStationIRState::FAR_RIGHT, RobotDockingState::FAR_RIGHT}
                };

  // Check if robot is in charge
  if(charger) {
    return RobotDockingState::IN_DOCK;
  }

  for(unsigned int i = 0; i < 6; i++) {
    if(validateSignal(signal_filt, dock_signal_array[i][0])) 
    {
      return (RobotDockingState::State)dock_signal_array[i][1];
      
    }
  }
  return RobotDockingState::ERROR;
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
    if ( state==DONE ) setState(IDLE); // when this function is called after final state 'DONE'.
    if ( state==DOCKED_IN ) {
      if ( dock_stabilizer++ > 20 ) {
        is_enabled = false;
        can_run = false;
        setStateVel(DONE, 0.0, 0.0); break;
      }
      setStateVel(DOCKED_IN, 0.0, 0.0); break;
    }
    if ( bump_remainder > 0 ) {
      bump_remainder--;
      if ( charger ) { setStateVel(DOCKED_IN, 0.0, 0.0); break; } // when bumper signal is received early than charger(dock) signal.
      else {           setStateVel(BUMPED_DOCK, -0.01, 0.0); break; }
    } else if (state == BUMPED) {
      setState(IDLE); //should I remember and recall previous state?
      debug_str="how dare!!";
    }
    if ( bumper || charger ) {
      if( bumper && charger ) {
        bump_remainder = 0;
        setStateVel(BUMPED_DOCK, -0.01, 0.0); break;
      }
      if ( bumper ) {
        bump_remainder = 50;
        setStateVel(BUMPED, -0.05, 0.0); break;
      }
      if ( charger ) { // already docked in
        dock_stabilizer = 0;
        setStateVel(DOCKED_IN, 0.0, 0.0); break;
      }
    } else {
      if ( state==IDLE ) {
        dock_detector = 0;
        rotated = 0.0;
        setStateVel(SCAN, 0.00, 0.66); break;
      }
      if ( state==SCAN ) {
        rotated += pose_update.heading()/(2.0*M_PI);
        std::ostringstream oss;
        oss << "rotated: " << std::fixed << std::setprecision(2) << std::setw(4) << rotated;
        debug_str = oss.str();
        if( std::abs(rotated) > 1.6 ) {
          setStateVel(FIND_STREAM, 0.0, 0.0); break;
        }
        if (  signal_filt[1]&(FAR_LEFT  + NEAR_LEFT )) dock_detector--;
        if (  signal_filt[1]&(FAR_RIGHT + NEAR_RIGHT)) dock_detector++;
        if ( (signal_filt[1]&FAR_CENTER) || (signal_filt[1]&NEAR_CENTER) ) {
          setStateVel(ALIGNED, 0.05, 0.00); break;
        } else if ( signal_filt[1] ) {
          setStateVel(SCAN, 0.00, 0.10); break;
        } else {
          setStateVel(SCAN, 0.00, 0.66); break;
        }

      } else if (state==ALIGNED || state==ALIGNED_FAR || state==ALIGNED_NEAR) {
        if ( signal_filt[1] ) {
          if ( signal_filt[1]&NEAR )
          {
            if ( ((signal_filt[1]&NEAR) == NEAR_CENTER) || ((signal_filt[1]&NEAR) == NEAR) ) { setStateVel(ALIGNED_NEAR, 0.05,  0.0); debug_str = "AlignedNearCenter"; break; }
            if (   signal_filt[1]&NEAR_LEFT  ) {                                               setStateVel(ALIGNED_NEAR, 0.05,  0.1); debug_str = "AlignedNearLeft"  ; break; }
            if (   signal_filt[1]&NEAR_RIGHT ) {                                               setStateVel(ALIGNED_NEAR, 0.05, -0.1); debug_str = "AlignedNearRight" ; break; }
          }
          if ( signal_filt[1]&FAR )
          {
            if ( ((signal_filt[1]&FAR) == FAR_CENTER) || ((signal_filt[1]&FAR) == FAR) ) { setStateVel(ALIGNED_FAR, 0.1,  0.0); debug_str = "AlignedFarCenter"; break; }
            if (   signal_filt[1]&FAR_LEFT  ) {                                            setStateVel(ALIGNED_FAR, 0.1,  0.3); debug_str = "AlignedFarLeft"  ; break; }
            if (   signal_filt[1]&FAR_RIGHT ) {                                            setStateVel(ALIGNED_FAR, 0.1, -0.3); debug_str = "AlignedFarRight" ; break; }
          }
          dock_detector = 0;
          rotated = 0.0;
          setStateVel(SCAN, 0.00, 0.66); break;
        } else {
          debug_str = "lost signals";
          setStateVel(LOST, 0.00, 0.00); break;
        }
      } else if (state==FIND_STREAM) {
        if (dock_detector > 0 ) { // robot is placed in right side of docking station
          //turn  right , negative direction til get right signal from left sensor
          if (signal_filt[2]&(FAR_RIGHT+NEAR_RIGHT)) {
            setStateVel(GET_STREAM, 0.05, 0.0); break;
          } else {
            setStateVel(FIND_STREAM, 0.0, -0.33); break;
          }
        } else if (dock_detector < 0 ) { // robot is placed in left side of docking station
          //turn left, positive direction till get left signal from right sensor
          if (signal_filt[0]&(FAR_LEFT+NEAR_LEFT)) {
            setStateVel(GET_STREAM, 0.05, 0.0); break;
          } else {
            setStateVel(FIND_STREAM, 0.0, 0.33); break;
          }
        }
      } else if (state==GET_STREAM) {
        if (dock_detector > 0) { //robot is placed in right side of docking station
          if (signal_filt[2]&(FAR_LEFT+NEAR_LEFT)) {
            dock_detector = 0;
            rotated = 0.0;
            setStateVel(SCAN, 0.0, 0.10); break;
          } else {
            setStateVel(GET_STREAM, 0.05, 0.0); break;
          }
        } else if (dock_detector < 0) { // robot is placed in left side of docking station
          if (signal_filt[0]&(FAR_RIGHT+NEAR_RIGHT)) {
            dock_detector = 0;
            rotated = 0.0;
            setStateVel(SCAN, 0.0, 0.10); break;
          } else {
            setStateVel(GET_STREAM, 0.05, 0.0); break;
          }
        }
      } else {
        dock_detector = 0;
        rotated = 0.0;
        setStateVel(SCAN, 0.00, 0.66); break;
      }
    }
    setStateVel(UNKNOWN, 0.00, 0.00); break;
  } while(0);
}
*/
} // kobuki namespace
