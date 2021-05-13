# Just Control (jcontrol) && Just Math (jmath)
A collection of controls and math code. Fun.

Thoughts / ToDos:
- Combine A, B dynamics function into single function returning a pair of matrices?
    this would reduce recalculations in discretized continuous dynamics
- make TimeRange::time_step.size be named dt instead.
- add dynamics_type attribute to dynamics - (be continuous or discrete)
- create problem phrasing that can send different problems (eg: single goal vs traj tracking) to ilqr solver
- re-org arguments to quick-cost function. I'm thinking Qf first?
- make sure typedefs are consistent across all types, reduce redundancy - I have a lot of typedefs haha
- notes about how dt() function is defined for discrete dynamics. attribute to note whether discrete or not?
- clean up templates to maximize template deduction / code readability
    programs using library can typedef the main stuff out- make the lib code readable!
- add documentation / tests haha