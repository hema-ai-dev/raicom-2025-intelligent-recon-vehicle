# Robot code

> This directory is a historical source reference, not a turnkey ROS workspace. It has not been rebuilt after curation and depends on missing hardware drivers, device configuration, and runtime details.

## Packages

- `nav_demo`: map, navigation parameters, mission script, detector and `GunSweep.srv`.
- `car_bringup`: chassis odometry, serial bridge and localization configuration.

The vehicle model package (`mowen`) is not included here because it contains competition-supplied material; it is kept in the private archive only.

Hardware drivers are external dependencies and are not vendored in this showcase.

## Historical fidelity

`mission_controller.py` retains route and tuning values from the 2025 vehicle backup. Cleanup replaced machine-specific model/output paths and removed one duplicate service call, but did not redesign the route. Its active archived path contains only part of the route and has the gun invocations commented out. It has not been run on the original vehicle after cleanup.

`prototype/auto_aim_controller.py` is a reference-only record of the depth-camera aiming experiment. Its external YOLOv5 runtime, default data configuration, and calibration file are not included, and compatibility with the packaged target weight has not been revalidated. It is intentionally separated from the final sweep service.

## Safety

The sweep service starts unarmed. Supplying `~armed:=true` is a deliberate hardware action and must only happen in a controlled area after verifying the controller protocol and emergency stop.
