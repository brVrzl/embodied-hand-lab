# External reference audit

These repositories were inspected for adapter patterns only. No third-party
controller or hand mapping is copied into the training run.

- `Physical-Intelligence/openpi`: commit
  `15a9616a00943ada6c20a0f158e3adb39df2ccac`; the official JAX LoRA path and
  current `pi05_base` fine-tuning configuration are used.
- `unitreerobotics/unitree_lerobot`: commit
  `41c2805742de879ddab2d8d6beaeaf215f876395`; its LeRobot conversion and
  six-motor Inspire examples were inspected. Its hand channel order is
  Unitree-specific and is not substituted for this project's RH56 order.
  Repository: <https://github.com/unitreerobotics/unitree_lerobot>.
- `EmptyBlueBox/DexLatent`: commit
  `502750df34b9fa6a1b56a66481586887ec722f9c`; Inspire representations and
  cross-hand latent design were inspected. No latent representation is used
  in this baseline. Repository: <https://github.com/EmptyBlueBox/DexLatent>.

The derived adapter keeps the project's audited six active RH56 actuator
channels and native absolute arm targets. The external references are not
runtime dependencies and their code is not vendored.
