# JAKA Gate 3B Stage 3 State-Preparation Review

Date: 2026-07-16  
Scope: documentation and static inspection only  
Hardware connection: not performed  
Robot-state changes: none

## Decision summary

The blocked Stage 3 precondition was incorrect. For the installed JAKA SDK
V2.2.7, `is_in_servomove` reports the controller's **servo-position-control
mode**, not the robot's electrical power/enable state. The version-specific
release notes explicitly require `edg_init(true, ...)` **before**
`servo_move_enable(true)`. Consequently, servo-move mode is not a prerequisite
for EDG initialization or for an EDG state-only read.

No verified JAKA App or teach-pendant procedure was found for manually entering
SDK servo-move mode. The official App manual exposes servo-position mode as a
reported motion-mode value, but does not document an operator control for
entering it. The probe must not treat ordinary robot enable as equivalent to
servo-move mode.

The recommended revised Stage 3 remains state-preserving: require
`is_in_servomove == false`, enter EDG, perform one `edg_get_stat`, leave EDG,
and log out. It must not call `servo_move_enable` at all. A later cyclic-command
stage would require separate authorization for the disposable native process to
own a paired servo-mode enable/disable lifecycle.

## Evidence reviewed

### Installed V2.2.7 package

The installed package contains the C/C++ headers, x86-64 and AArch64 shared
libraries, and English/Chinese release-note PDFs. It contains no bundled source
examples. The x86-64 library identifies itself as
`V2.2.7stable_linux` and exports `servo_move_enable`,
`is_in_servomove`, `edg_init`, and `edg_get_stat`.

Version-specific sources:

- [`JAKAZuRobot.h`](../../../../third_party/jaka_sdk/v2.2.7/linux/c_cpp/inc_of_c++/JAKAZuRobot.h)
  declares `servo_move_enable(BOOL)`, describes it as enabling/disabling servo
  mode, and describes `is_in_servomove(BOOL*)` as reporting whether the robot is
  in servo-move mode. It states that `servo_j` and `edg_servo_j` work only when
  that mode is active.
- [`jakaAPI.h`](../../../../third_party/jaka_sdk/v2.2.7/linux/c_cpp/inc_of_c/jakaAPI.h)
  provides the equivalent C capability boundary.
- [`SDK V2.2.7 Release Notes.pdf`](../../../../third_party/jaka_sdk/v2.2.7/SDK%20V2.2.7%20Release%20Notes.pdf)
  specifies controller version `1_7_2_28` or newer, Linux outside a virtual
  machine, a real-time patch where practical, an approximately 8 ms client
  cycle, CPU affinity/priority/frequency precautions, and the ordering
  `edg_init` before `servo_move_enable`. It also requires EDG initialization to
  be disabled after EDG servo use.
- Static symbol inspection shows that the public servo-mode setter maps to an
  internal `enable_servo_move` operation. It does not establish undocumented
  controller-side physical semantics.

Repository evidence:

- [`native/jaka_servo_worker/main.cpp`](../../../../native/jaka_servo_worker/main.cpp)
  already follows the required enable order: EDG initialization, then servo-mode
  enable. Its cleanup uses reverse acquisition order: servo-mode disable, EDG
  disable, logout.
- [`src/jaka_driver_adapter/servo_jog.py`](../../../../src/jaka_driver_adapter/servo_jog.py)
  uses the correct enable order but disables servo mode without disabling EDG.
  That wrapper is not an acceptable lifecycle reference for Gate 3B cleanup.
- The existing Stage 3 entry-only binary excludes `servo_move_enable`,
  `edg_servo_j`, `edg_servo_p`, and general motion APIs at link time. Its
  conservative requirement that servo-move mode already be active is the item
  that needs revision; its command exclusion remains correct.

### Official JAKA documentation

The [official C++ SDK reference](https://www.jaka.com/docs/en/guide/V3/SDK/cpp.html)
defines `is_in_servomove` as the servo-motion-state query and
`servo_move_enable(true/false)` as entry/exit for servo position control. It
also states that EDG must be initialized before EDG interfaces are used and
that ordinary `servo_j`/`servo_p` are unavailable while EDG is enabled. Its EDG
example calls `edg_init(true, ...)` before `servo_move_enable(true)`.

The [official Python SDK reference](https://www.jaka.com/docs/en/guide/1.7.2/SDK/python.html)
states that servo commands bypass controller trajectory interpolation, require
continuous user-planned targets, and must be followed by
`servo_move_enable(false)` to leave position-control mode.

The [official JAKA App software user manual](https://www.jaka.com/profile/upload/2026/03/31/20260331144921A043.pdf)
lists servo-position mode as motion-mode value 4 in a read-only input-register
inventory. A full-text search found no documented App/pendant action for
entering `servo_move`/servo-position mode. It documents power, robot enable,
jog, drag, and automatic/manual operations, which are different modes.

## Findings by review question

1. **Meaning of `is_in_servomove`:** it reports whether the controller is in
   servo position-control (servo-move) mode. It does not report motor power or
   ordinary robot enable. Gate 3B physically observed `enabled=true` and
   `is_in_servomove=false`, confirming that distinction.

2. **Lifecycle and side effects of `servo_move_enable`:** `true` enters and
   `false` exits servo position-control mode. The call contains no joint or
   Cartesian target, so it is not itself a pose command. It nevertheless changes
   the controller's motion-control mode, changes which motion APIs are accepted,
   and is therefore a write-capable safety transition. The installed header does
   not document hold behavior or guarantee that entry is physically inert.

3. **EDG prerequisite:** servo-move mode must **not** be enabled before EDG
   initialization. V2.2.7 explicitly orders `edg_init` first. EDG initialization
   is the prerequisite for `edg_get_stat`; the documentation does not require
   servo-move mode merely to read EDG state.

4. **Manual preparation:** no supported manual App/pendant control was found.
   An operator can safely prepare normal power/robot-enable/fault-free state,
   but that does not prepare SDK servo-move mode. Until JAKA supplies contrary
   version-specific instructions, manual servo-mode preparation must be treated
   as unavailable.

5. **Physical effect of enable alone:** no target is supplied and JAKA describes
   motion as resulting from subsequent `servo_j`/`servo_p` or EDG servo calls.
   However, no inspected source promises that the mode transition cannot hold,
   stop, or otherwise affect control behavior. It must not be classified as a
   harmless read-only preparation call.

6. **Disable responsibility:** any process that successfully enables servo-move
   mode must explicitly disable it during every cleanup path. Logout is not a
   documented substitute. EDG must also be explicitly disabled. A process that
   did not create a pre-existing mode must not silently take ownership of it.

7. **Safe ordering:** JAKA's motion examples generally show login, power/robot
   enable, EDG initialization, servo-mode enable, EDG operation, and explicit
   teardown. In this project, power and ordinary robot enable remain operator
   responsibilities and are only verified by the probe. For a future command
   stage, the proposed process-owned order is:

   1. operator prepares power, normal robot enable, E-stop access, clear
      workspace, and fault-free state;
   2. disposable native process logs in and verifies all state, including an
      initially inactive servo-move mode;
   3. capture the invariant joint vector;
   4. `edg_init(true, local_ip)`;
   5. `servo_move_enable(true)` and verify `is_in_servomove == true`;
   6. issue only the separately authorized bounded EDG operation;
   7. cease all target writes;
   8. `servo_move_enable(false)` and verify inactive where a read remains safe;
   9. `edg_init(false, local_ip)`;
   10. logout and terminate the disposable process.

   The online examples are inconsistent about the relative order of the two
   cleanup calls, and the V2.2.7 release notes do not prescribe it. Reverse
   acquisition order is recommended because it removes servo command acceptance
   before dismantling EDG. Cleanup must still attempt both operations and logout
   if either call fails, while preserving the first error.

8. **V2.2.7-specific constraints:** the installed API has only the one-argument
   `servo_move_enable(BOOL)` and two-argument `edg_init(BOOL, const char*)`.
   Current online V3 documentation adds blocking/robot-ID and EDG port/mode
   parameters that are unavailable here. The installed package requires
   controller `1_7_2_28` or newer; the exact controller firmware remains
   unreported by Gate 3A. The release-note sentence naming “SDK V2.2.2” is an
   internal typo: its title, compatibility notes, exported library string, and
   remainder consistently identify V2.2.7.

## Documentation differences and unresolved facts

- Current V3 signatures must not be copied into the V2.2.7 probe.
- JAKA's current reference says controller versions after V20 make
  `servo_move_enable` blocking, but the installed V2.2.7 signature does not
  expose the newer `is_block` selection. Its real call latency and blocking
  behavior therefore remain to be measured only after explicit authorization.
- Official examples use both EDG-first and servo-first teardown order. No
  version-specific guarantee resolves this; cleanup must be defensive.
- Neither the installed headers nor official App manual guarantees zero physical
  effect from servo-mode entry alone.
- The controller firmware version was unavailable through Gate 3A's installed
  read APIs, so compatibility with the release-note minimum remains an operator
  or vendor-verification item.

## Recommended operator preparation

For a revised state-only Stage 3, the operator should prepare only the normal
robot conditions already used in Gate 3A/Stage 2: emergency stop accessible,
workspace clear, robot powered and normally enabled through the approved
operator interface, no fault/E-stop/collision, and tool/user IDs 0/0. The
operator should not attempt to force servo-move mode through an undocumented
pendant sequence.

The probe should require `is_in_servomove == false` at entry. A true value would
indicate an unexpected external owner and must abort the attempt.

## Revised Stage 3 authorization proposal

Revise Stage 3 to the following exact, still non-commanding sequence:

1. build and statically re-verify the entry-only binary;
2. connect/login and repeat the approved read-only preflight;
3. require fault 0, E-stop false, collision false, powered/enabled true,
   tool/user 0/0, valid six-radian joint vector, and
   `is_in_servomove == false`;
4. call `edg_init(true, 192.168.71.19)`;
5. call `edg_get_stat` exactly once;
6. call `edg_init(false, 192.168.71.19)`;
7. logout and terminate the disposable process.

This revision calls neither form of `servo_move_enable` and issues zero joint or
Cartesian commands. It tests only EDG transport entry, one feedback read, and
exit. Stage 4 remains blocked: before any invariant target may be sent, a
separate approval must assign paired `servo_move_enable(true/false)` ownership
to the disposable native process and accept the mode-transition risk described
above.
