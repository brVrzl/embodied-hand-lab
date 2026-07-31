# JAKA kinematic support

This package contains the MuJoCo-backed continuation IK and shared conservative
joint-limit helpers used before an `AcceptedArmTarget` is produced.

It is not a physical JAKA transport. The only maintained Quest/JAKA physical
output boundary is `teleoperation.jaka.JakaAcceptedJointTargetAdapter` feeding
the sole native EDG worker. The removed Python SDK/ServoJog compatibility stack
must not be recreated as a parallel command path.
