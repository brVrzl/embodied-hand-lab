#include "trajectory.hpp"
#include "../jaka_readonly_diagnostic/readonly_backend.hpp"

#include <algorithm>
#include <array>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
constexpr double pi=3.14159265358979323846, displacement=5*pi/180, duration=5, hold=1, settle=.5;
constexpr double warning=.2*pi/180, delay=.150, base_hard=.75*pi/180, non_target=.1*pi/180;
constexpr double margin=5*pi/180, period=.008, rapid_step=.25*pi/180, observed_speed_abort=3.5*pi/180;
constexpr std::array<double,6> lo{-2*pi,-125*pi/180,-130*pi/180,-2*pi,-120*pi/180,-2*pi};
constexpr std::array<double,6> hi{ 2*pi, 125*pi/180, 130*pi/180, 2*pi, 120*pi/180, 2*pi};
struct O{bool physical=false;std::string ip,out;int tool=0,user=0;};
std::string val(int&i,int n,char**v){if(++i>=n)throw std::runtime_error("missing value");return v[i];}
O parse(int n,char**v){O o;for(int i=1;i<n;++i){std::string a=v[i];if(a=="--physical-hardware")o.physical=true;else if(a=="--robot-ip")o.ip=val(i,n,v);else if(a=="--result-file")o.out=val(i,n,v);else if(a=="--expected-tool-id")o.tool=std::stoi(val(i,n,v));else if(a=="--expected-user-frame-id")o.user=std::stoi(val(i,n,v));else throw std::runtime_error("unknown option: "+a);}if(!o.physical)return o;in_addr x{};if(inet_pton(AF_INET,o.ip.c_str(),&x)!=1||o.out.empty())throw std::runtime_error("explicit IP/output required");return o;}
void vec(std::ostream&o,const std::array<double,6>&q){o<<'[';for(size_t i=0;i<6;++i){if(i)o<<',';o<<q[i];}o<<']';}
int run(const O&o){if(!o.physical){std::cout<<"{\"stage\":\"dry-run\",\"connection_opened\":false,\"commands_issued\":0}\n";return 0;}auto b=jaka_readonly::make_vendor_backend();jaka_readonly::State s{};jaka_readonly::Batch batch{};b->query_sdk_version(s,batch);if(b->connect(o.ip,batch))throw std::runtime_error("login failed");b->read_static(s,batch);b->read_fast(s,batch);b->read_slow(s,batch);std::cerr<<"READ-ONLY +5 DEGREE PLAN — no EDG or command. Preliminary joints: ";vec(std::cerr,s.joint_position_rad);std::cerr<<"\nFresh capture in 3...\n";for(int i=3;i>0;--i){std::cerr<<i<<"...\n";std::this_thread::sleep_for(std::chrono::seconds(1));}jaka_readonly::State fresh=s;jaka_readonly::Batch fresh_batch{};bool fast=b->read_fast(fresh,fresh_batch);b->read_slow(fresh,fresh_batch);
 std::string outcome="completed";int rc=0;if(!fast||!fresh.joint_position_available||!std::all_of(fresh.joint_position_rad.begin(),fresh.joint_position_rad.end(),[](double x){return std::isfinite(x);}))outcome="invalid_joint_state",rc=2;else if(!fresh.status_available||fresh.fault_code||!fresh.powered||!fresh.enabled||fresh.emergency_stop||fresh.collision)outcome="unsafe_controller_state",rc=2;else if(fresh.tool_id!=o.tool||fresh.user_frame_id!=o.user)outcome="frame_mismatch",rc=2;
 auto target=fresh.joint_position_rad;target[5]+=displacement;if(target[5]>hi[5]-margin||target[5]<lo[5]+margin)outcome="joint_limit_margin",rc=2;
 double pv=0,pa=0,pj=0,peak_hard=base_hard;for(int i=0;i<=5000;++i){auto x=jaka_gate3c::septic_state(0,displacement,duration,i*duration/5000);pv=std::max(pv,std::abs(x.velocity));pa=std::max(pa,std::abs(x.acceleration));pj=std::max(pj,std::abs(x.jerk));peak_hard=std::max(peak_hard,2.5*std::abs(x.velocity)*delay);}int logout=b->disconnect(batch);if(logout)outcome="logout_failure",rc=2;
 std::ofstream out(o.out);if(!out)throw std::runtime_error("cannot open result");out<<std::setprecision(12)<<"{\n\"schema_version\":\"jaka_gate3c_5deg_plan.v1\",\n\"outcome\":\""<<outcome<<"\",\n\"edg_entered\":false,\n\"commands_issued\":0,\n\"selected_joint\":\"jaka_joint_6\",\n\"direction\":\"positive\",\n\"fresh_start\":";vec(out,fresh.joint_position_rad);out<<",\n\"outward_target\":";vec(out,target);out<<",\n\"return_target\":";vec(out,fresh.joint_position_rad);out<<",\n\"displacement_rad\":"<<displacement<<",\n\"displacement_deg\":5,\n\"outward_duration_s\":5,\n\"hold_duration_s\":1,\n\"return_duration_s\":5,\n\"settling_duration_s\":"<<settle<<",\n\"period_s\":"<<period<<",\n\"planned_peak_velocity_rad_s\":"<<pv<<",\n\"planned_peak_acceleration_rad_s2\":"<<pa<<",\n\"planned_peak_jerk_rad_s3\":"<<pj<<",\n\"tracking_warning_rad\":"<<warning<<",\n\"configured_observation_delay_s\":"<<delay<<",\n\"dynamic_hard_formula\":\"max(0.75deg,2.5*abs(commanded_velocity)*0.150s)\",\n\"dynamic_hard_min_rad\":"<<base_hard<<",\n\"dynamic_hard_peak_rad\":"<<peak_hard<<",\n\"hard_crossing_persistence\":2,\n\"rapid_divergence_step_rad\":"<<rapid_step<<",\n\"target_joint_envelope_lower_rad\":"<<fresh.joint_position_rad[5]-pi/180<<",\n\"target_joint_envelope_upper_rad\":"<<fresh.joint_position_rad[5]+6*pi/180<<",\n\"non_target_joint_envelope_rad\":"<<non_target<<",\n\"observed_speed_abort_rad_s\":"<<observed_speed_abort<<",\n\"start_safe_lower_margin_rad\":"<<fresh.joint_position_rad[5]-(lo[5]+margin)<<",\n\"target_safe_upper_margin_rad\":"<<(hi[5]-margin)-target[5]<<",\n\"tcp_effect_estimate\":\"5 degree joint-6 axial rotation; approximately zero axial tool-origin translation; <=8.724 mm sweep for a point 100 mm from axis; RH56/cables rotate 5 degrees\",\n\"sdk_version\":\""<<fresh.sdk_version<<"\",\n\"tool_id\":"<<fresh.tool_id<<",\n\"user_id\":"<<fresh.user_frame_id<<",\n\"logout_code\":"<<logout<<"\n}\n";return rc;}
}
int main(int n,char**v){try{return run(parse(n,v));}catch(const std::exception&e){std::cerr<<"error: "<<e.what()<<'\n';return 64;}}
