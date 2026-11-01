# Changelog

## Provenance clarification

- Documented in `NOTICE.md` that the baseline ROS packages (`car_bringup`, `nav_demo`) were provided together with the vehicle hardware by the competition organizer, and that the team's work covers the tuned parameters, surveyed waypoints and mission scripts.
- Replaced the inaccurate `Proprietary` license tag in both ROS `package.xml` files with a reference to `NOTICE.md`, since those packages build on the organizer-provided baseline.

## Presentation clarification

- Reframed the repository as an engineering case study and curated archive rather than a reproducible robot stack.
- Added a component status table and a summary of the repository owner's primary contributions.
- Clarified that the retained mission path is partial and that its gun-service calls are commented out.
- Marked the closed-loop aiming controller as a reference-only prototype with missing external runtime and calibration assets.
- Removed the unverified claim that the packaged YOLO11 target weight is directly compatible with the retained YOLOv5-style prototype.

## GitHub showcase edition

- Split the 2025 national-competition project from the 2024 and provincial-competition archives.
- Removed reports, organizer documents, raw datasets, personal information and photos containing people.
- Removed generated build trees, duplicate framework copies, obsolete maps and third-party driver source.
- Replaced machine-specific model paths with repository-relative defaults.
- Reduced the soldier detector to its active implementation and documented the historical label mapping.
- Added validation and fail-safe defaults to the water-gun sweep service.
- Cleaned catkin metadata and retained only the final map selected by the archived launch configuration.
- Added model cards, architecture notes, reproducibility boundaries and an automated repository validator.

These changes were made for archival clarity. They have not been revalidated on the original vehicle.
