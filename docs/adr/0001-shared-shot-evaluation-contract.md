---
status: accepted
---
# Share a shot evaluation contract between calculator and robot

Use the current working TypeScript calculator as the reference for Hub and passback evaluation, with shared fixtures and final compensated-trajectory validation in both languages. The Java solver already includes release prediction and motion compensation, but treating a radial-map score as final trajectory validity can accept a miss; explicit feasibility and command-source results are preferred over treating heuristic robustness as scoring probability. Keep the legacy solver as an explicit rollback path rather than an automatic bypass of rejected shots.
