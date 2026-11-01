# Publication Checklist

English | [简体中文](PUBLICATION_CHECKLIST.zh-CN.md)

- [ ] Confirm that all contributors agree to publish the code and model weights.
- [ ] Confirm the repository name, description, and attribution wording.
- [ ] Select a license for project-authored code, or intentionally keep the repository without a top-level `LICENSE`.
- [ ] Install Git LFS and confirm that all three `.pt` files are committed as LFS objects.
- [ ] Confirm that no competition report, organizer-provided vehicle model, unattributed organizer material, raw dataset, or photo containing people is included; the retained ROS baseline packages must remain attributed in `NOTICE.md`.
- [ ] Confirm that the GitHub Actions `validate` workflow passes.
- [ ] Do not describe retained validation metrics as independent test-set or competition-field accuracy.
- [ ] Do not describe the depth-camera aiming prototype as the final competition strategy.
- [ ] Keep the launcher controller disarmed by default and retain the safety warning.
- [ ] Before adding a demonstration video, verify permission for all people, locations, and audio appearing in it.
