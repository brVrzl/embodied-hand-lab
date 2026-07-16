#include "trajectory.hpp"
#include "../jaka_readonly_diagnostic/readonly_backend.hpp"

#include <algorithm>
#include <array>
#include <arpa/inet.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDisplacementRad = 0.25 * kPi / 180.0;
constexpr double kMarginRad = 5.0 * kPi / 180.0;
constexpr double kOutwardDurationS = 2.0;
constexpr double kHoldDurationS = 0.4;
constexpr double kReturnDurationS = 2.0;
constexpr double kPeriodS = 0.008;
constexpr double kVelocityLimit = 0.005;
constexpr double kAccelerationLimit = 0.010;
constexpr double kJerkLimit = 0.040;
constexpr double kTrackingErrorLimit = 0.0005;
constexpr std::size_t kJointIndex = 5;
constexpr std::array<double, 6> kLower{-2*kPi, -125*kPi/180, -130*kPi/180, -2*kPi, -120*kPi/180, -2*kPi};
constexpr std::array<double, 6> kUpper{ 2*kPi,  125*kPi/180,  130*kPi/180,  2*kPi,  120*kPi/180,  2*kPi};

struct Options {
  bool physical = false, estop = false, workspace = false, no_person = false;
  bool cables = false, direction = false, ready = false;
  std::string robot_ip, approval, output;
  int expected_tool = 0, expected_user = 0;
};

std::string value(int& i, int argc, char** argv) {
  if (++i >= argc) throw std::runtime_error("missing option value");
  return argv[i];
}

Options parse(int argc, char** argv) {
  Options o;
  for (int i=1;i<argc;++i) {
    const std::string a=argv[i];
    if(a=="--physical-hardware")o.physical=true; else if(a=="--estop-access-confirmed")o.estop=true;
    else if(a=="--workspace-clear-confirmed")o.workspace=true; else if(a=="--no-person-in-workspace-confirmed")o.no_person=true;
    else if(a=="--cable-clearance-confirmed")o.cables=true; else if(a=="--direction-understood")o.direction=true;
    else if(a=="--ready-to-interrupt")o.ready=true; else if(a=="--robot-ip")o.robot_ip=value(i,argc,argv);
    else if(a=="--stage-approval")o.approval=value(i,argc,argv); else if(a=="--result-file")o.output=value(i,argc,argv);
    else if(a=="--expected-tool-id")o.expected_tool=std::stoi(value(i,argc,argv));
    else if(a=="--expected-user-frame-id")o.expected_user=std::stoi(value(i,argc,argv));
    else throw std::runtime_error("unknown option: "+a);
  }
  if (!o.physical) return o;
  in_addr ip{};
  if(!o.estop||!o.workspace)
    throw std::runtime_error("physical plan requires E-stop and workspace confirmations");
  if(o.approval!="I_APPROVE_GATE3C_STAGE_1_PLAN")throw std::runtime_error("incorrect Stage 3C-1 approval");
  if(inet_pton(AF_INET,o.robot_ip.c_str(),&ip)!=1||o.output.empty())throw std::runtime_error("explicit IP and result file required");
  return o;
}

void joints(std::ostream& out,const std::array<double,6>& q){out<<'[';for(size_t i=0;i<6;++i){if(i)out<<',';out<<q[i];}out<<']';}

int execute(const Options& o) {
  if(!o.physical){std::cout<<"{\"stage\":\"dry-run\",\"connection_opened\":false,\"edg_entered\":false,\"commands_issued\":0}\n";return 0;}
  auto backend=jaka_readonly::make_vendor_backend(); jaka_readonly::State state{}; jaka_readonly::Batch batch{};
  backend->query_sdk_version(state,batch);
  if(backend->connect(o.robot_ip,batch)!=0)throw std::runtime_error("login failed");
  backend->read_static(state,batch); const bool fast=backend->read_fast(state,batch); backend->read_slow(state,batch);
  int rc=0; std::string outcome="completed";
  if(!fast||!state.joint_position_available||!std::all_of(state.joint_position_rad.begin(),state.joint_position_rad.end(),[](double x){return std::isfinite(x);})) {outcome="invalid_joint_state";rc=2;}
  else if(!state.status_available||state.fault_code!=0||!state.powered||!state.enabled||state.emergency_stop||state.collision){outcome="unsafe_controller_state";rc=2;}
  else if(!state.tool_id_available||!state.user_frame_id_available||state.tool_id!=o.expected_tool||state.user_frame_id!=o.expected_user){outcome="frame_mismatch";rc=2;}
  std::array<double,6> target=state.joint_position_rad; target[kJointIndex]+=kDisplacementRad;
  const double safe_lo=kLower[kJointIndex]+kMarginRad, safe_hi=kUpper[kJointIndex]-kMarginRad;
  if(rc==0&&(target[kJointIndex]<safe_lo||target[kJointIndex]>safe_hi)){outcome="joint_limit_margin";rc=2;}
  double max_v=0,max_a=0,max_j=0;
  for(int i=0;i<=2000;++i){auto s=jaka_gate3c::septic_state(0,kDisplacementRad,kOutwardDurationS,i*kOutwardDurationS/2000);max_v=std::max(max_v,std::abs(s.velocity));max_a=std::max(max_a,std::abs(s.acceleration));max_j=std::max(max_j,std::abs(s.jerk));}
  if(max_v>kVelocityLimit||max_a>kAccelerationLimit||max_j>kJerkLimit){outcome="trajectory_limit_violation";rc=2;}
  const int logout=backend->disconnect(batch); if(logout!=0){outcome="logout_failure";rc=2;}
  std::ofstream out(o.output);
  if (!out) throw std::runtime_error("cannot open result file");
  out<<std::setprecision(12);
  out<<"{\n  \"schema_version\":\"jaka_gate3c_plan.v1\",\n  \"stage\":\"3C-1-plan\",\n  \"physical_execution\":true,\n  \"outcome\":\""<<outcome<<"\",\n";
  out<<"  \"edg_entered\":false,\n  \"servo_mode_changed\":false,\n  \"commands_issued\":0,\n  \"selected_joint_index_1_based\":6,\n  \"selected_joint_name\":\"jaka_joint_6\",\n  \"direction\":\"positive\",\n";
  out<<"  \"start_joint_rad\":";joints(out,state.joint_position_rad);out<<",\n  \"outward_target_rad\":";joints(out,target);out<<",\n  \"return_target_rad\":";joints(out,state.joint_position_rad);out<<",\n";
  out<<"  \"displacement_rad\":"<<kDisplacementRad<<",\n  \"displacement_deg\":0.25,\n  \"start_safe_lower_margin_rad\":"<<state.joint_position_rad[kJointIndex]-safe_lo<<",\n  \"start_safe_upper_margin_rad\":"<<safe_hi-state.joint_position_rad[kJointIndex]<<",\n  \"target_safe_lower_margin_rad\":"<<target[kJointIndex]-safe_lo<<",\n  \"target_safe_upper_margin_rad\":"<<safe_hi-target[kJointIndex]<<",\n";
  out<<"  \"period_s\":"<<kPeriodS<<",\n  \"outward_duration_s\":"<<kOutwardDurationS<<",\n  \"hold_duration_s\":"<<kHoldDurationS<<",\n  \"return_duration_s\":"<<kReturnDurationS<<",\n  \"expected_total_motion_s\":"<<(kOutwardDurationS+kHoldDurationS+kReturnDurationS)<<",\n";
  out<<"  \"velocity_limit_rad_s\":"<<kVelocityLimit<<",\n  \"acceleration_limit_rad_s2\":"<<kAccelerationLimit<<",\n  \"jerk_limit_rad_s3\":"<<kJerkLimit<<",\n  \"planned_peak_velocity_rad_s\":"<<max_v<<",\n  \"planned_peak_acceleration_rad_s2\":"<<max_a<<",\n  \"planned_peak_jerk_rad_s3\":"<<max_j<<",\n  \"tracking_error_abort_rad\":"<<kTrackingErrorLimit<<",\n";
  out<<"  \"tcp_effect_estimate\":\"joint-6 axis rotation: approximately zero tool-origin translation for an axial tool frame; 0.25 degree orientation; <=0.436 mm sweep for a point 100 mm from axis\",\n  \"sdk_version\":\""<<state.sdk_version<<"\",\n  \"tool_id\":"<<state.tool_id<<",\n  \"user_id\":"<<state.user_frame_id<<",\n  \"logout_code\":"<<logout<<"\n}\n";
  std::cerr<<"GATE 3C-1 READ-ONLY PLAN: joint 6 positive 0.25 deg; no EDG, servo change, or command.\n";
  return rc;
}
}
int main(int argc,char**argv){try{return execute(parse(argc,argv));}catch(const std::exception&e){std::cerr<<"configuration error: "<<e.what()<<'\n';return 64;}}
