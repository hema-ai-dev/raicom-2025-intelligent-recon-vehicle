# Engineering Decisions

English | [简体中文](DESIGN_DECISIONS.zh-CN.md)

## Why closed-loop aiming was not the final strategy

The depth-camera aiming pipeline reached a working prototype, but a working prototype was not automatically the best competition solution for this vehicle. The final decision reflected four constraints:

1. Limited onboard performance caused slower responses when several vision and ROS processes ran together.
2. Switching into the water-gun workload introduced visible stalls and could leave SLAM unresponsive afterward.
3. Continuous depth-camera and inference use reduced the available battery margin.
4. The sweep strategy was less precise but more deterministic and exposed fewer failure modes.

Both paths are preserved: `robot/prototype/` demonstrates the closed-loop experiment, while `gun_sweep_server.py` represents the final engineering choice.

## Why the raw datasets are not published

The datasets may include people, competition locations, and material whose redistribution permission was never confirmed. They are also unsuitable for a normal Git repository due to their size. This repository publishes configuration examples, training arguments, validation records, and aggregate plots instead.

## Why third-party ROS drivers are not vendored

Hardware drivers and navigation components have their own versions, upstream maintainers, and licenses. Copying entire historical dependencies would blur the project boundary and preserve outdated build assumptions. The public edition records the dependency names and interfaces while leaving installation to the respective upstream projects.

## Why there is no one-command launch script

The complete system depends on the physical chassis, serial-device mapping, sensor models, calibration, and competition map. The original vehicle is not available for a new end-to-end verification. The repository therefore documents module entry points and reproduction limits instead of presenting an unverified one-command launch procedure.
