# Model Card

English | [简体中文](MODEL_CARD.zh-CN.md)

## Files

| File | Historical role | SHA-256 |
|---|---|---|
| `soldier/final_training_best.pt` | Best weight from the last retained soldier-board training run; corresponds to the published training metrics | `1C84949A79C15036730BD4A87877BDF1E9A947D9EE87FB33937803DB161ADCD4` |
| `soldier/robot_deployed_best.pt` | Soldier-board weight explicitly referenced by the vehicle backup | `78F8D2E91C832CBFCAA1AD56888F8DC1E939AD94F33AF5A88AF53B9C9C43EC7A` |
| `target/best.pt` | Target-center weight preserved from the same target-detection R&D stage as the depth-camera experiment; compatibility with the retained controller has not been revalidated | `8FEC08A1ACA0BDA03E6FFC723B574B2CF577565311507A31043870923B99E166` |

## Tasks and class semantics

- The final soldier-board training configuration retained four class slots, including two entries named `Enemy`. The deployed vehicle weight uses `red / white / blue / colorful`, which the runtime maps to `enemy / hostage / friend`.
- The target model contains one class: `target_center`. The retained controller uses a YOLOv5-style runtime, while this weight came from a YOLO11 training record; this archive does not claim that they can be used together without reconstruction work.

The two soldier weights have different hashes and different retained class records. This repository deliberately does not present them as the same model version.

## Metric boundary

The metrics shown in the main README come from the last validation epoch in the retained training logs. There is no independent test set, cross-validation result, or current-environment rerun, so the values must not be interpreted as guaranteed generalization, competition-field accuracy, or a safety measure.

## Intended use

These weights are retained for competition engineering archives, education, and offline demonstrations. They are not intended for identity recognition, surveillance, law enforcement, or any high-risk automated decision.

## Known risks

- The raw datasets are not public, so class balance, annotation quality, and source permissions cannot be fully audited here.
- The external YOLOv5 runtime and calibration data used by the aiming experiment are absent, and compatibility with the retained target weight has not been rerun.
- New cameras, lighting, backgrounds, and printed target materials may introduce substantial domain shift.
- The models can miss or misclassify targets and must never be the only safety condition for a projectile launcher.
