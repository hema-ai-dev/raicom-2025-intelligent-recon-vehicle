# Robot code

## Packages

- `nav_demo`: map, navigation parameters, mission script, detector and `GunSweep.srv`.
- `car_bringup`: chassis odometry, serial bridge and localization configuration.
- `mowen`: URDF, meshes and RViz configuration.

Hardware drivers are external dependencies and are not vendored in this showcase.

## Historical fidelity

`mission_controller.py` retains the route and tuning values from the 2025 vehicle backup. Cleanup replaced machine-specific model/output paths and removed one duplicate service call, but did not redesign the route. It has not been run on the original vehicle after cleanup.

`prototype/auto_aim_controller.py` documents the depth-camera aiming experiment. It is intentionally separated from the final sweep service.

## Safety

The sweep service starts unarmed. Supplying `~armed:=true` is a deliberate hardware action and must only happen in a controlled area after verifying the controller protocol and emergency stop.
