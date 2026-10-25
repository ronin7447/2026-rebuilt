# Original hood-edge-free fit snapshot

These files preserve the initial no-hood-edge-penalty polynomial evaluated during the comparison. The current generator removes BOTH hood-edge and distance penalties. Regenerated active base coefficients now use those selection rules.

Robot runtime now bounds out-of-range mapped hood angles to the nearest requested limit and solves a new speed using radial and sideways release velocity. It validates the final trajectory and speed limits, rather than rejecting solely because the polynomial overshoots a hood bound. Tilt-corrected commands must still satisfy local mechanism bounds. The 63-degree far-map trial remains selected and blends at 4.9–5.5 m.

The calculator webpage does not yet reproduce the robot's selected far trial or hood-limit recovery. Comparison JSON/Markdown created earlier are historical evaluations before this recovery behavior and should not be interpreted as current end-to-end robot predictions.
