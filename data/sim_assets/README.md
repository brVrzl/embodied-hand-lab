# Simulation Assets

This directory contains the MuJoCo assets required by `data/sim_assets/jaka_rh56.xml`.

- `meshes/jaka_minicobo_meshes/Link0.STL` through `Link6.STL` are JAKA MiniCobo visual meshes sourced from the public `KunSong-L/Jaka-Minicobo-Sim-Env` repository and copied locally so MuJoCo does not depend on old absolute workstation paths.
- `meshes/rh56/*.STL` are right-hand RH56 meshes sourced from the vendor URDF materials under the original uploaded RH56 archive.

The XML also includes analytic collision geoms for simulation/contact checks; mesh geoms are visual-only where marked with `contype="0" conaffinity="0"`.
