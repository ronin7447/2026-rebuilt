---
status: accepted
---
# Continue aiming through vision occlusion

Defense frequently blocks AprilTags, so accepted-vision age alone warns rather than vetoes automatic feeding while fused odometry continues. Continuously gate automatic feeding on trajectory and mechanism readiness, with a deliberate manual override and a constants switch; this favors continued operation through occlusion while accepting that collision or wheel-slip pose errors can still cause misses. Preserve this intentional policy in telemetry and tests so future changes do not silently restore a fresh-vision requirement.
