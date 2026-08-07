# RGB-D interface

`vision_interface` defines the repository's RGB-D frame contract and optional
Intel RealSense adapter. Importing it does not open a camera.

## Frame contract

`CameraInterface.capture()` returns one `RGBDFrame` from one source frameset:

- `rgb`: `uint8[H,W,3]`, RGB channel order;
- `depth_m`: `float32[H,W]` in metres, with zero for invalid depth;
- intrinsics corresponding to the depth pixel geometry;
- host receive time plus colour/depth device timestamps and frame numbers;
- the device timestamp domains;
- whether depth is aligned to colour.

Device timestamps are compared only when their domains match. The adapter
rejects excessive same-domain colour/depth skew during startup. Code must not
manufacture synchronization by subtracting timestamps from different domains.

RealSense optical coordinates are `+X` right, `+Y` down, and `+Z` forward.
`depth_to_point_cloud()` does not guess a robot transform and rejects
unrectified nonzero distortion. An explicit, calibrated transform is required
before a point can be described in a robot or workspace frame.

## Current status

- Mock-frame capture, same-frameset structure, depth processing, and atomic
  episode integration have offline tests.
- RealSense configuration and diagnostic tools exist.
- The versioned D435 serials are site snapshots, not portable defaults.
- Dual-D435 physical synchronization, camera-to-robot calibration, and
  end-to-end physical collection are not validated.
- Experimental RGB-guided fill is not a geometry authority.

Inspect current tools without opening a camera:

```bash
.venv/bin/python tools/check_realsense_stream.py --help
.venv/bin/python tools/process_rgbd_tabletop.py --help
```

Opening or enumerating a physical camera requires an explicit device operation.
Installation and device troubleshooting are documented in
`docs/setup/INSTALLATION.md` and `docs/TROUBLESHOOTING.md`; schema and
collection semantics are in `docs/data/`.
