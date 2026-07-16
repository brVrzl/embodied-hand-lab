# Legacy teleoperation boundary

Everything in `src/teleop_tools` is historical/prototype teleoperation code. It
may depend on HEBI and is not an implementation source for the clean-slate JAKA
foundation in `src/teleoperation` and `native/jaka_servo_worker`.

Do not import this package from the new arm-only runtime. The historical files
remain available for provenance and comparison; migration is by replacement at
the composition boundary, not incremental refactoring.
