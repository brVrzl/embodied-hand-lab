# Known limitations

- The shared output-acceleration feasibility correction at current HEAD is
  offline tested but not physically validated.
- The prior J4 collision alarm's cause is unresolved; payload mismatch was
  corrected by the operator but is not proven to be the sole cause.
- Controller-health fault propagation is covered offline. The sole-session
  lightweight polling timing path completed a bounded physical run, but an
  actual induced controller collision/estop event was not used as a validation
  method.
- TCP1–TCP10 are recorded as zero; no completed TCP calibration is claimed.
- Quest-driven physical RH56 teleoperation is not validated in the shared path.
- Physical JAKA full-envelope translation/orientation is only partially
  validated and must not be expanded from historical small/bounded results.
- Quest Unity/APK/runtime version and current headset installation remain
  external facts; repository source audits do not prove the deployed build.
- The digital-twin workspace remains “Integrated Workspace,” not “Simulation
  Ready”; its documented failed trajectories and calibration tasks remain.
- HEBI and iPhone paths remain parallel/compatibility workflows and do not
  share every Quest/JAKA contract.
- Vendor reference sources are retained as supplied and are not necessarily
  importable project modules.
