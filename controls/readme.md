# Just Control (jcontrol) && Just Math (jmath)
A collection of controls and math code. Fun.

Thoughts:
- Combine A, B dynamics function into single function returning a pair of matrices?
    this would reduce recalculations in discretized continuous dynamics
- make TimeRange::time_step.size be named dt instead.
- add dynamics_type attribute to dynamics - (be continuous or discrete)