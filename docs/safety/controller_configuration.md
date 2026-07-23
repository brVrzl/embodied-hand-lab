# Controller configuration boundary

The controller is the authority for payload, center of mass, installation,
TCP, collision settings, speed limits, and other safety parameters. Repository
configuration may document expectations, but the Quest/JAKA path must not
write these values automatically.

Latest operator-supplied record:

- payload 0.8 kg;
- COM `[9.289, 12.427, 36.961]` mm;
- upright/floor installation, X=0°, Z=0°;
- TCP1–TCP10 zero;
- controller safety limits unchanged.

Before a future authorized physical test, read/confirm the actual controller
state through the approved procedure. Do not “correct” a mismatch in software
or at the controller without a separate engineering decision and explicit
authorization.

TCP remaining zero is a known limitation: it is not proof of calibrated
tool-center geometry. Do not present pose accuracy at an uncalibrated tool
frame as a completed TCP validation.
