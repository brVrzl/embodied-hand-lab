#include "trajectory.hpp"
#include "../jaka_zero_motion_probe/zero_motion_backend.hpp"

#include <algorithm>
#include <array>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock=std::chrono::steady_clock;
constexpr double kPi=3.14159265358979323846,kD=.25*kPi/180.,kT=2.,kHold=.4,kDt=.008;
constexpr double kV=.005,kA=.010,kJ=.040,kTrack=.0005,kFirst=.0001,kMargin=5*kPi/180.;
constexpr std::size_t kJoint=5,kOutSteps=250,kHoldSteps=50,kReturnSteps=250,kCount=551;
constexpr double kD5=5*kPi/180.,kT5=5.,kHold5=1.,kSettle5=.5,kWarn5=.2*kPi/180.,kBaseHard5=.75*kPi/180.,kDelay5=.150,kNonTarget5=.1*kPi/180.,kRapid5=.25*kPi/180.,kSpeed5=3.5*kPi/180.;
constexpr std::uint64_t kSoftPeriod=8'800'000,kHardPeriod=12'000'000,kWakeDebt=8'000'000;
constexpr std::array<double,6> kLo{-2*kPi,-125*kPi/180,-130*kPi/180,-2*kPi,-120*kPi/180,-2*kPi};
constexpr std::array<double,6> kHi{ 2*kPi, 125*kPi/180, 130*kPi/180, 2*kPi, 120*kPi/180, 2*kPi};
std::atomic<bool> stop_requested{false}; void signal_handler(int){stop_requested.store(true,std::memory_order_relaxed);}

struct Options{bool help=false,vendor=false,physical=false,estop=false,workspace=false,no_person=false,cables=false,direction=false,ready=false,five=false,fake_deterministic_clock=false;std::string ip,edg,output,csv;int tool=0,user=0;double fake_tracking_offset=0,fake_tracking_growth=0,fake_non_target_offset=0;jaka_zero::FakeOptions fake{};};
std::string val(int&i,int n,char**v){if(++i>=n)throw std::runtime_error("missing option value");return v[i];}
Options parse(int n,char**v){Options o;for(int i=1;i<n;++i){std::string a=v[i];
 if(a=="--help"||a=="-h")o.help=true;
 else if(a=="--backend"){auto x=val(i,n,v);o.vendor=x=="vendor";if(x!="vendor"&&x!="fake")throw std::runtime_error("bad backend");}
 else if(a=="--physical-hardware")o.physical=true;else if(a=="--five-degree-profile")o.five=true;else if(a=="--estop-access-confirmed")o.estop=true;else if(a=="--workspace-clear-confirmed")o.workspace=true;else if(a=="--no-person-in-workspace-confirmed")o.no_person=true;else if(a=="--cable-clearance-confirmed")o.cables=true;else if(a=="--direction-understood")o.direction=true;else if(a=="--ready-to-interrupt")o.ready=true;
 else if(a=="--robot-ip")o.ip=val(i,n,v);else if(a=="--edg-state-ip")o.edg=val(i,n,v);else if(a=="--result-file")o.output=val(i,n,v);else if(a=="--trajectory-csv")o.csv=val(i,n,v);else if(a=="--expected-tool-id")o.tool=std::stoi(val(i,n,v));else if(a=="--expected-user-frame-id")o.user=std::stoi(val(i,n,v));
 else if(a=="--fake-deterministic-clock")o.fake_deterministic_clock=true;else if(a=="--fake-command-failure-cycle")o.fake.command_failure_cycle=std::stoull(val(i,n,v));else if(a=="--fake-read-failure-cycle")o.fake.read_failure_cycle=std::stoull(val(i,n,v));else if(a=="--fake-observed-delta-rad")o.fake.observed_joint_delta_rad=std::stod(val(i,n,v));else if(a=="--fake-tracking-offset-rad")o.fake_tracking_offset=std::stod(val(i,n,v));else if(a=="--fake-tracking-growth-rad")o.fake_tracking_growth=std::stod(val(i,n,v));else if(a=="--fake-non-target-offset-rad")o.fake_non_target_offset=std::stod(val(i,n,v));else if(a=="--fake-servo-enable-failure")o.fake.servo_enable_failure=true;else if(a=="--fake-servo-disable-failure")o.fake.servo_disable_failure=true;else throw std::runtime_error("unknown option: "+a);}
 if(o.help)return o;
 if(o.vendor&&o.fake_deterministic_clock)throw std::runtime_error("--fake-deterministic-clock is forbidden with the vendor backend");
 if(!o.vendor)return o;
 in_addr x{},y{};
 if(!o.physical||!o.estop||!o.workspace||!o.no_person||!o.cables||!o.direction||!o.ready)throw std::runtime_error("all physical confirmations required");
 if(inet_pton(AF_INET,o.ip.c_str(),&x)!=1||inet_pton(AF_INET,o.edg.c_str(),&y)!=1||o.output.empty()||o.csv.empty())throw std::runtime_error("explicit addresses and outputs required");
 return o;}

void print_help(){std::cout<<
"Usage: jaka_gate3c_motion_probe [options]\n"
"\n"
"Offline inspection:\n"
"  --help, -h                         Show this help without creating a backend.\n"
"  --backend fake                     Use the offline fake backend (default).\n"
"  --fake-deterministic-clock         Use a logical 8 ms clock with the fake backend only.\n"
"\n"
"Physical execution (all gates are mandatory):\n"
"  --backend vendor --physical-hardware\n"
"  --robot-ip IPV4 --edg-state-ip IPV4\n"
"  --expected-tool-id ID --expected-user-frame-id ID\n"
"  --estop-access-confirmed --workspace-clear-confirmed\n"
"  --no-person-in-workspace-confirmed --cable-clearance-confirmed\n"
"  --direction-understood --ready-to-interrupt\n"
"  --result-file PATH --trajectory-csv PATH\n"
"\n"
"The default physical profile moves only J6 by +0.25 degree and returns.\n"
"--five-degree-profile selects the explicit +5 degree profile.\n";}

struct PlanPoint{jaka_zero::Joints q{},v{},a{};const char*phase="";};
std::vector<PlanPoint> make_plan(const jaka_zero::Joints&start,bool five){const double d=five?kD5:kD,t=five?kT5:kT,h=five?kHold5:kHold;const size_t steps=static_cast<size_t>(std::llround(t/kDt)),holds=static_cast<size_t>(std::llround(h/kDt)),settles=five?static_cast<size_t>(std::llround(kSettle5/kDt)):0;std::vector<PlanPoint> p(steps+1+holds+steps+settles);size_t n=0;
 for(size_t i=0;i<=steps;++i){auto s=jaka_gate3c::septic_state(start[kJoint],d,t,i*kDt);p[n].q=start;p[n].q[kJoint]=s.position;p[n].v[kJoint]=s.velocity;p[n].a[kJoint]=s.acceleration;p[n++].phase="outward";}
 for(size_t i=0;i<holds;++i){p[n].q=start;p[n].q[kJoint]+=d;p[n++].phase="hold";}
 for(size_t i=1;i<=steps;++i){auto s=jaka_gate3c::septic_state(start[kJoint]+d,-d,t,i*kDt);p[n].q=start;p[n].q[kJoint]=s.position;p[n].v[kJoint]=s.velocity;p[n].a[kJoint]=s.acceleration;p[n++].phase="return";}
 for(size_t i=0;i<settles;++i){p[n].q=start;p[n++].phase="settle";}
 if(n!=p.size())throw std::runtime_error("internal plan size");
 return p;}

struct Row{std::uint64_t period=0,wake=0,read=0,command=0;int read_code=0,command_code=0;double tracking=0,expected_lag=0,dynamic_hard=0;bool tracking_warning=false,hard_crossing=false;PlanPoint plan{};jaka_zero::Joints observed{};};
struct Result{std::string outcome="completed",trace;int exit_code=0,initial_cpu=-1,final_cpu=-1;size_t commands=0,warnings=0,hard=0,period_misses=0,completion_misses=0,tracking_warnings=0,hard_crossings=0,cpu_migrations=0;double peak_tracking=0,outward_observed=0,return_error=0,max_non_target=0,loop_duration_s=0,process_cpu_s=0;std::uint64_t max_period=0,max_wake=0,max_command=0;jaka_zero::PreflightState pre{};jaka_zero::Joints start{},outward{},final_obs{};jaka_zero::TimedResult login{},preflight{},entry{},initial_read{},enable{},disable{},exit{},logout{};};
struct Stats{size_t count=0;double mean=0,median=0,stddev=0,min=0,max=0,p95=0,p99=0;};
double percentile(const std::vector<std::uint64_t>&v,double q){if(v.empty())return 0;const double x=q*static_cast<double>(v.size()-1);const size_t lo=static_cast<size_t>(x),hi=std::min(lo+1,v.size()-1);return v[lo]+(x-lo)*static_cast<double>(v[hi]-v[lo]);}
Stats stats(std::vector<std::uint64_t>v){Stats s;s.count=v.size();if(v.empty())return s;std::sort(v.begin(),v.end());const long double sum=std::accumulate(v.begin(),v.end(),static_cast<long double>(0));s.mean=static_cast<double>(sum/v.size());long double sq=0;for(auto x:v){const long double d=x-s.mean;sq+=d*d;}s.median=percentile(v,.5);s.stddev=std::sqrt(static_cast<double>(sq/v.size()));s.min=v.front();s.max=v.back();s.p95=percentile(v,.95);s.p99=percentile(v,.99);return s;}
void write_stats(std::ostream&o,const Stats&s){o<<"{\"count\":"<<s.count<<",\"mean_ns\":"<<s.mean<<",\"median_ns\":"<<s.median<<",\"stddev_ns\":"<<s.stddev<<",\"min_ns\":"<<s.min<<",\"max_ns\":"<<s.max<<",\"p95_ns\":"<<s.p95<<",\"p99_ns\":"<<s.p99<<'}';}
bool safe(const Options&o,const jaka_zero::PreflightState&s,std::string&r){if(s.fault_code||!s.powered||!s.enabled)r="controller_state";else if(s.emergency_stop)r="estop";else if(s.collision)r="collision";else if(s.servo_move_active)r="external_servo_owner";else if(s.tool_id!=o.tool||s.user_frame_id!=o.user)r="frame";else if(!jaka_zero::finite_joints(s.captured_joint_rad))r="nonfinite";else return true;return false;}
void write_vec(std::ostream&o,const jaka_zero::Joints&q){o<<'[';for(size_t i=0;i<6;++i){if(i)o<<',';o<<q[i];}o<<']';}
bool validate_plan(const std::vector<PlanPoint>&plan,const jaka_zero::Joints&start,double displacement){
 if(plan.empty()||jaka_zero::maximum_absolute_delta(plan.front().q,start)!=0.0)return false;
 for(const auto&point:plan){if(!jaka_zero::finite_joints(point.q)||!jaka_zero::finite_joints(point.v)||!jaka_zero::finite_joints(point.a))return false;for(size_t j=0;j<6;++j){if(j!=kJoint&&(point.q[j]!=start[j]||point.v[j]!=0.0||point.a[j]!=0.0))return false;}const double delta=point.q[kJoint]-start[kJoint];if(delta<-1e-15||delta>displacement+1e-15)return false;}
 return jaka_zero::maximum_absolute_delta(plan.back().q,start)==0.0;
}

int execute(const Options&o){if(o.help){print_help();return 0;}if(!o.vendor&&!o.physical&&o.output.empty()){std::cout<<"{\"mode\":\"fake_requires_output\",\"commands_issued\":0}\n";return 0;}const double d=o.five?kD5:kD;stop_requested=false;signal(SIGINT,signal_handler);signal(SIGTERM,signal_handler);
 auto b=o.vendor?jaka_zero::make_vendor_backend():jaka_zero::make_fake_backend(o.fake);Result r;r.login=b->initialize_and_login(o.ip);if(r.login.code){r.outcome="login_failure";r.exit_code=2;}
 if(!r.exit_code){r.preflight=b->preflight(r.pre);std::string why;if(r.preflight.code||!safe(o,r.pre,why)){r.outcome=r.preflight.code?"preflight_sdk_failure":why;r.exit_code=2;}}
 if(!r.exit_code&&o.vendor){std::cerr<<(o.five?"GATE 3C +5 DEG PLAN\nJoint: jaka_joint_6; direction: positive; displacement: 5 deg (0.08726646259971647 rad)\nDurations: 5 s outward, 1 s hold, 5 s return, 0.5 s settling\nRequested period: 8 ms; planned commands: 1439\nSeptic peaks: velocity 2.1875 deg/s, acceleration 1.50264 deg/s^2, jerk 2.1 deg/s^3\nTracking warning: 0.2 deg; dynamic hard threshold: 0.75-0.8203125 deg\nJoint-6 observation envelope: fresh start -1 deg through fresh start +6 deg\nNon-target observation envelope: fresh start +/-0.1 deg; observed-speed envelope: 3.5 deg/s\nCleanup: stop commands -> servo_move_enable(false) -> edg_init(false) -> logout -> process exit\n":"GATE 3C-2 PLAN: joint 6 +0.25 deg.\n")<<"Preliminary start: ";write_vec(std::cerr,r.pre.captured_joint_rad);std::cerr<<"\nCountdown 3...\n";for(int i=3;i>0;--i){std::cerr<<i<<"...\n";std::this_thread::sleep_for(std::chrono::seconds(1));}}
 if(!r.exit_code){r.preflight=b->precommand_check(r.pre);std::string why;if(r.preflight.code||!safe(o,r.pre,why)){r.outcome=r.preflight.code?"precommand_sdk_failure":why;r.exit_code=2;}else{r.start=r.pre.captured_joint_rad;r.outward=r.start;r.outward[kJoint]+=d;if(r.outward[kJoint]>kHi[kJoint]-kMargin){r.outcome="joint_limit";r.exit_code=2;}}}
 std::vector<PlanPoint> plan;if(!r.exit_code){plan=make_plan(r.start,o.five);if(!validate_plan(plan,r.start,d)){r.outcome="trajectory_validation_failure";r.exit_code=2;}}std::vector<Row> rows(plan.size());
 if(!r.exit_code&&o.vendor){std::cerr<<"Fresh start: ";write_vec(std::cerr,r.start);std::cerr<<"\nOutward target: ";write_vec(std::cerr,r.outward);std::cerr<<"\nOnly joint 6 delta: +"<<d<<" rad.\n";}
 if(!r.exit_code){r.entry=b->enter_edg(o.edg);if(r.entry.code){r.outcome="edg_entry_failure";r.exit_code=2;}}
 jaka_zero::EdgObservation obs{};if(!r.exit_code){r.initial_read=b->read_edg(obs);if(r.initial_read.code||jaka_zero::maximum_absolute_delta(obs.joint_position_rad,r.start)>kFirst){r.outcome="first_command_guard";r.exit_code=2;}}
 if(!r.exit_code){r.enable=b->enable_servo_move();if(r.enable.code){r.outcome="servo_enable_failure";r.exit_code=2;}}
 auto scheduled=Clock::now(),previous=scheduled;const auto loop_start=scheduled;const std::clock_t cpu_start=std::clock();size_t consecutive_miss=0,consecutive_hard=0;double previous_tracking=0;jaka_zero::Joints previous_observed=obs.joint_position_rad;const size_t outward_end=static_cast<size_t>(std::llround((o.five?kT5:kT)/kDt));int previous_cpu=-1;
 for(size_t i=0;!r.exit_code&&i<plan.size();++i){if(stop_requested){r.outcome="operator_interrupted";r.exit_code=130;break;}auto begin=o.fake_deterministic_clock?scheduled:Clock::now();auto wake=begin>scheduled?static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(begin-scheduled).count()):0;rows[i].wake=wake;r.max_wake=std::max(r.max_wake,rows[i].wake);
  const int cpu=sched_getcpu();if(i==0)r.initial_cpu=cpu;if(previous_cpu>=0&&cpu>=0&&cpu!=previous_cpu)++r.cpu_migrations;previous_cpu=cpu;r.final_cpu=cpu;
  if(i){rows[i].period=std::chrono::duration_cast<std::chrono::nanoseconds>(begin-previous).count();r.max_period=std::max(r.max_period,rows[i].period);if(rows[i].period>kHardPeriod||wake>=kWakeDebt){r.outcome="hard_timing_miss";r.hard++;r.exit_code=2;break;}if(rows[i].period>kSoftPeriod){r.period_misses++;r.warnings++;if(++consecutive_miss>=2){r.outcome="repeated_timing_miss";r.hard++;r.exit_code=2;break;}}else consecutive_miss=0;}previous=begin;
  auto rd=b->read_edg(obs);rows[i].read=rd.duration_ns;rows[i].read_code=rd.code;if(!o.vendor){obs.joint_position_rad[kJoint]+=o.fake_tracking_offset+o.fake_tracking_growth*static_cast<double>(i);obs.joint_position_rad[0]+=o.fake_non_target_offset;}rows[i].observed=obs.joint_position_rad;rows[i].plan=plan[i];if(rd.code||!jaka_zero::finite_joints(obs.joint_position_rad)){r.outcome="edg_read_failure";r.exit_code=2;break;}
  const auto&reference=i?plan[i-1]:plan[0];rows[i].tracking=std::abs(obs.joint_position_rad[kJoint]-reference.q[kJoint]);r.peak_tracking=std::max(r.peak_tracking,rows[i].tracking);
  if(o.five){rows[i].expected_lag=std::abs(reference.v[kJoint])*kDelay5;rows[i].dynamic_hard=std::max(kBaseHard5,2.5*rows[i].expected_lag);rows[i].tracking_warning=rows[i].tracking>kWarn5;if(rows[i].tracking_warning)r.tracking_warnings++;rows[i].hard_crossing=rows[i].tracking>rows[i].dynamic_hard;if(rows[i].hard_crossing){r.hard_crossings++;if(++consecutive_hard>=2){r.outcome="persistent_dynamic_tracking";r.exit_code=2;break;}}else consecutive_hard=0;if(i&&rows[i].tracking>kWarn5&&rows[i].tracking-previous_tracking>=kRapid5){r.outcome="rapid_tracking_divergence";r.exit_code=2;break;}for(size_t j=0;j<5;++j){r.max_non_target=std::max(r.max_non_target,std::abs(obs.joint_position_rad[j]-r.start[j]));if(std::abs(obs.joint_position_rad[j]-r.start[j])>kNonTarget5){r.outcome="non_target_envelope";r.exit_code=2;break;}}if(r.exit_code)break;if(obs.joint_position_rad[kJoint]<r.start[kJoint]-kPi/180||obs.joint_position_rad[kJoint]>r.start[kJoint]+6*kPi/180){r.outcome="target_joint_envelope";r.exit_code=2;break;}if(i&&rows[i].period){double observed_speed=std::abs(obs.joint_position_rad[kJoint]-previous_observed[kJoint])/(rows[i].period*1e-9);if(observed_speed>kSpeed5){r.outcome="observed_speed_envelope";r.exit_code=2;break;}}previous_tracking=rows[i].tracking;previous_observed=obs.joint_position_rad;}
  else if(jaka_zero::maximum_absolute_delta(obs.joint_position_rad,reference.q)>kTrack){r.outcome="tracking_error";r.exit_code=2;break;}
  if(i<=outward_end&&obs.joint_position_rad[kJoint]<r.start[kJoint]-kFirst){r.outcome="wrong_direction";r.exit_code=2;break;}
  auto cmd=b->command_invariant(plan[i].q);rows[i].command=cmd.duration_ns;rows[i].command_code=cmd.code;r.max_command=std::max(r.max_command,rows[i].command);if(cmd.code){r.outcome="command_failure";r.exit_code=2;break;}r.commands++;
  auto complete=o.fake_deterministic_clock?begin:Clock::now();auto next=scheduled+std::chrono::nanoseconds(8'000'000);if(complete>next){r.completion_misses++;r.warnings++;if(complete>scheduled+std::chrono::nanoseconds(kHardPeriod)){r.outcome="hard_completion_miss";r.hard++;r.exit_code=2;break;}scheduled=complete+std::chrono::nanoseconds(8'000'000);}else scheduled=next;if(i+1<plan.size()&&!o.fake_deterministic_clock)std::this_thread::sleep_until(scheduled);
 }
 r.loop_duration_s=std::chrono::duration<double>(Clock::now()-loop_start).count();r.process_cpu_s=static_cast<double>(std::clock()-cpu_start)/CLOCKS_PER_SEC;
 if(!r.exit_code){jaka_zero::EdgObservation final{};auto x=b->read_edg(final);if(x.code){r.outcome="final_read_failure";r.exit_code=2;}else{r.final_obs=final.joint_position_rad;r.outward_observed=0;for(size_t i=0;i<r.commands;++i)r.outward_observed=std::max(r.outward_observed,rows[i].observed[kJoint]-r.start[kJoint]);r.return_error=jaka_zero::maximum_absolute_delta(r.final_obs,r.start);}}
 if(b->servo_move_active())r.disable=b->disable_servo_move();
 if(r.disable.code){r.outcome="servo_disable_failure";r.exit_code=2;}
 if(b->edg_active())r.exit=b->exit_edg();
 if(r.exit.code){r.outcome="edg_exit_failure";r.exit_code=2;}
 if(b->logged_in())r.logout=b->logout();
 if(r.logout.code){r.outcome="logout_failure";r.exit_code=2;}
 r.trace=b->lifecycle_trace();
 std::ofstream csv(o.csv);if(!csv)throw std::runtime_error("cannot open csv");csv<<std::setprecision(17)<<"index,phase,period_ns,wake_ns,read_ns,command_ns,raw_tracking_error_rad,expected_lag_error_rad,dynamic_hard_threshold_rad,tracking_warning,hard_crossing";for(int j=1;j<=6;++j)csv<<",cmd_q"<<j<<",cmd_v"<<j<<",cmd_a"<<j<<",obs_q"<<j;csv<<'\n';for(size_t i=0;i<r.commands;++i){auto&x=rows[i];csv<<i<<','<<x.plan.phase<<','<<x.period<<','<<x.wake<<','<<x.read<<','<<x.command<<','<<x.tracking<<','<<x.expected_lag<<','<<x.dynamic_hard<<','<<x.tracking_warning<<','<<x.hard_crossing;for(size_t j=0;j<6;++j)csv<<','<<x.plan.q[j]<<','<<x.plan.v[j]<<','<<x.plan.a[j]<<','<<x.observed[j];csv<<'\n';}
 std::vector<std::uint64_t>periods,wakes,reads,commands;periods.reserve(r.commands?r.commands-1:0);wakes.reserve(r.commands);reads.reserve(r.commands);commands.reserve(r.commands);for(size_t i=0;i<r.commands;++i){if(i)periods.push_back(rows[i].period);wakes.push_back(rows[i].wake);reads.push_back(rows[i].read);commands.push_back(rows[i].command);}const auto period_stats=stats(periods),wake_stats=stats(wakes),read_stats=stats(reads),command_stats=stats(commands);
 std::ofstream out(o.output);if(!out)throw std::runtime_error("cannot open result");out<<std::setprecision(17)<<"{\n\"schema_version\":\"jaka_gate3c_motion.v2\",\n\"profile\":\""<<(o.five?"joint6_plus_5deg":"joint6_plus_0.25deg")<<"\",\n\"outcome\":\""<<r.outcome<<"\",\n\"commands\":"<<r.commands<<",\n\"planned_commands\":"<<plan.size()<<",\n\"requested_period_ns\":8000000,\n\"timing_clock\":\""<<(o.fake_deterministic_clock?"deterministic_fake":"monotonic_wall")<<"\",\n\"loop_duration_s\":"<<r.loop_duration_s<<",\n\"process_cpu_s\":"<<r.process_cpu_s<<",\n\"process_cpu_percent\":"<<(r.loop_duration_s>0?r.process_cpu_s/r.loop_duration_s*100:0)<<",\n\"initial_cpu\":"<<r.initial_cpu<<",\n\"final_cpu\":"<<r.final_cpu<<",\n\"cpu_migrations\":"<<r.cpu_migrations<<",\n\"peak_tracking_error_rad\":"<<r.peak_tracking<<",\n\"tracking_warning_threshold_rad\":"<<(o.five?kWarn5:kTrack)<<",\n\"tracking_warning_crossings\":"<<r.tracking_warnings<<",\n\"dynamic_hard_base_rad\":"<<(o.five?kBaseHard5:kTrack)<<",\n\"configured_observation_delay_s\":"<<(o.five?kDelay5:0)<<",\n\"hard_threshold_crossings\":"<<r.hard_crossings<<",\n\"target_joint_observation_envelope_rad\":["<<(r.start[kJoint]-(o.five?kPi/180:kTrack))<<','<<(r.start[kJoint]+(o.five?6*kPi/180:kTrack))<<"],\n\"non_target_observation_envelope_rad\":"<<(o.five?kNonTarget5:kTrack)<<",\n\"maximum_non_target_observation_delta_rad\":"<<r.max_non_target<<",\n\"actual_outward_displacement_rad\":"<<r.outward_observed<<",\n\"final_return_error_rad\":"<<r.return_error<<",\n\"timing_warnings\":"<<r.warnings<<",\n\"hard_timing_misses\":"<<r.hard<<",\n\"period_misses\":"<<r.period_misses<<",\n\"completion_misses\":"<<r.completion_misses<<",\n\"timing\":{\"start_to_start_period\":";write_stats(out,period_stats);out<<",\"wake_lateness\":";write_stats(out,wake_stats);out<<",\"edg_state_read\":";write_stats(out,read_stats);out<<",\"edg_command\":";write_stats(out,command_stats);out<<"},\n\"start\":";write_vec(out,r.start);out<<",\n\"outward_target\":";write_vec(out,r.outward);out<<",\n\"return_target\":";write_vec(out,r.start);out<<",\n\"final_observed\":";write_vec(out,r.final_obs);out<<",\n\"login_duration_ns\":"<<r.login.duration_ns<<",\n\"precommand_duration_ns\":"<<r.preflight.duration_ns<<",\n\"edg_entry_code\":"<<r.entry.code<<",\n\"edg_entry_duration_ns\":"<<r.entry.duration_ns<<",\n\"initial_edg_read_code\":"<<r.initial_read.code<<",\n\"initial_edg_read_duration_ns\":"<<r.initial_read.duration_ns<<",\n\"servo_enable_code\":"<<r.enable.code<<",\n\"servo_enable_duration_ns\":"<<r.enable.duration_ns<<",\n\"servo_disable_code\":"<<r.disable.code<<",\n\"servo_disable_duration_ns\":"<<r.disable.duration_ns<<",\n\"edg_exit_code\":"<<r.exit.code<<",\n\"edg_exit_duration_ns\":"<<r.exit.duration_ns<<",\n\"logout_code\":"<<r.logout.code<<",\n\"logout_duration_ns\":"<<r.logout.duration_ns<<",\n\"lifecycle\":\""<<r.trace<<"\",\n\"operator_observation\":\"pending\"\n}\n";return r.exit_code;}
}
int main(int n,char**v){try{return execute(parse(n,v));}catch(const std::exception&e){std::cerr<<"error: "<<e.what()<<'\n';return 64;}}
