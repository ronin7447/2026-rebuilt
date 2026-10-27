# Fixed-camera mounting analysis — 2026-10-03

Recommendation: prototype the approximately 0.40 m camera at 15° upward; a camera with its optical center at 22 in (0.5588 m) should start around 10° upward. Prefer a supported full-sensor image mode and complementary forward/rearward views for the stated Tower-to-Hub use case. These are geometric starting points, not a proven field-wide optimum. Do not use 25° upward for both general-localization cameras.

## Sources and assumptions

- [FIRST manual, sections 5.6 and 5.11](https://firstfrc.blob.core.windows.net/frc2026/Manual/HTML/2026GameManual.htm): physical tag heights, printed square dimensions and Trench clearance.
- [WPILib 2026.2.1 welded layout](https://raw.githubusercontent.com/wpilibsuite/allwpilib/v2026.2.1/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2026-rebuilt-welded.json): coordinates and facing directions. The included JSON was extracted from the installed WPILib 2026.2.1 JAR, matching the layout selected in this repository.
- [Current robot transforms](../../../src/main/java/frc/robot/constants/Constants.java): left camera at (0.0508, +0.3302, 0.4699) m, yaw +90°; right at (0.0508, -0.3302, 0.4318) m, yaw -90°; both currently level.
- The user reports a 70° horizontal OV9281 lens. All illustrative camera comparisons assume that FOV. The other camera/lens must be measured separately.
- Full 1280×800 sensor coverage gives an estimated 47.27° vertical FOV using an ideal pinhole model. A horizontal-preserving 1280×720 crop gives 43.00°. Actual calibration and crop behavior control the real answer. Resolution names alone do not specify FOV.
- Camera heights refer to the optical center above the floor, not the mounting plate or enclosure top.

## Actual tag heights

| Structure | Center height | IDs |
|---|---:|---|
| Tower | 0.55245 m / 21.75 in | 15, 16, 31, 32 |
| Outpost | 0.55245 m / 21.75 in | 13, 14, 29, 30 |
| Trench | 0.889 m / 35 in | 1, 6, 7, 12, 17, 22, 23, 28 |
| Hub | 1.12395 m / 44.25 in | remaining 16 IDs |

For a 0.40 m camera directly facing a tag, the center elevations are:

| Horizontal range | Tower/Outpost | Trench | Hub |
|---|---:|---:|---:|
| 1 m | 8.7° | 26.1° | 35.9° |
| 2 m | 4.4° | 13.7° | 19.9° |
| 4 m | 2.2° | 7.0° | 10.3° |
| 6 m | 1.5° | 4.7° | 6.9° |

Center visibility is insufficient: the corners and border need image space too. The calculations below use the entire 8.125 in printed square, conservatively including its outside border. Cropping only part of the outer border does not necessarily prevent every detection; these are full-square coverage limits, not measured detector cutoffs.

## Effect of tilt, height and crop

| Optical height / image coverage | Tilt | Lowest–highest elevation at image centerline |
|---|---:|---:|
| Either height, full 1280×800 | 10° | -13.64° to +33.64° |
| Either height, full 1280×800 | 15° | -8.64° to +38.64° |
| Either height, full 1280×800 | 20° | -3.64° to +43.64° |
| Either height, full 1280×800 | 25° | +1.36° to +48.64° |
| Either height, cropped 1280×720 | 25° | +3.50° to +46.50° |

For a camera at 0.40 m, a 25° mount clips the bottom of the full Tower/Outpost square beyond approximately 2.07 m with full 800-row coverage, or 0.80 m with the 720-row crop. The tag center can still be visible after its lower border has left the frame. At 15° or 20°, there is no equivalent far-distance clipping of those low tags on a level robot; resolution, incidence angle and obstructions still limit useful range.

At 22 in, the camera is 6.35 mm above the Tower/Outpost tag centers. A 25° mount therefore excludes even their centers for directly faced tags on a level robot. With full 1280×800 coverage, the whole low tag fits vertically starting at approximately:

| Upward tilt at 22 in | Minimum horizontal range for full low tag |
|---|---:|
| 5° | 0.32 m |
| 10° | 0.45 m |
| 15° | 0.72 m |
| 20° | 1.72 m |
| 25° | No full-square view |

At 22 in and 10°, a complete Hub tag fits vertically from about 1.00 m away. This supports 10° as a broad-coverage compromise, with 5° worth testing if very close low tags are valuable. At 0.40 m and 15°, a complete Hub tag fits from about 1.03 m; 20° improves this to about 0.87 m at the expense of low-tag/pitch margin.

These are ideal directly faced vertical limits. The projected four corners, horizontal FOV and tag-facing direction are also checked by the script. Robot pitch shifts the coverage: a camera aimed upward while climbing a bump loses low/distant tags more easily. The real bumper, shooter, intake, other robots and field structures can obstruct a geometrically eligible tag.

## Position on the actual field

Blue-origin coordinates are used, with +X toward the opposite alliance wall. The blue Hub's alliance-facing tags 25/26 are at X=4.022 m; Tower tags 31/32 are at X=0.008 m. Outpost tags 29/30 are near (0.008, 0.666/1.098) m. Alliance-facing Trench tags 28/23 are near (4.588, 0.644/7.425) m. The blue Hub center is near (4.626, 4.035) m; its front face must not be confused with its center.

The proposed pair below is illustrative: camera centers at robot X=+0.30/-0.30 m, Y=0; front camera 0.40 m high/15° up; rear camera 22 in high/10° up. Both use 70° horizontal and full 800-row coverage. Actual housing locations remain to be determined.

| Robot center, intake facing +X | Existing sideways cameras: eligible tags | Proposed front/rear pair: eligible tags | Meaning |
|---|---|---|---|
| Near Tower: (1.8, 4.0) m | Outpost 29, oblique | Hub 25/26 ahead; Tower 31/32 behind | Front/rear views directly address this use case. The current setup is not necessarily blind to every tag. |
| Near Hub: (3.2, 4.0) m | Trench 23/28 | Tower 31/32 behind | The forward camera is only about 0.52 m from the Hub face: the nearby Hub tags are too high. Looking back provides an alternate target. |
| Outpost-side lane: (3.2, 0.8) m | Several Hub tags | Trench 28 ahead; Outpost 29/30 behind | Low upward tilt retains useful wall tags. |
| Opposite lane: (3.2, 7.2) m | Several Hub tags | Trench 23 ahead | Front/rear is not universally superior; this location loses some sideways Hub views. |

At the near-Hub position, two 0.40 m-high cameras both tilted 25° have no complete-square candidates in the front/rear comparison: the nearby Hub tags are above the forward image and the distant Tower tags are clipped below the rear image. This occurs even with full 1280×800 coverage.

The script also checks headings 90°, 180° and 270°. The mixed pair retains the Hub/Tower alternatives at the near-Tower pose when the intake reverses, and Tower alternatives at the near-Hub pose. This is not continuous 360° coverage. In the neutral zone behind a Hub, that Hub can occlude the Tower; its opposite-face tags and Trench tags become relevant instead. The CSV includes an illustrative neutral-zone pose and rejects rays through approximate opaque Hub footprints.

Tag counts are not localization-quality scores: face-on size, blur, calibration, geometry, ambiguity and measurement age matter. A distant or grazing candidate can be much less useful than a nearby face-on one. Counts use an illustrative 6 m horizontal screening cutoff, not a claimed detection range. Tower uprights, trench arms, other robots and the robot's own geometry are not modeled.

## Resolution and physical mounting decision

Use 1280×800 if that exact camera/driver supports it at a practical processing rate. If reducing load, try a full-FOV mode such as 640×400 only after verifying it preserves the same scene; fewer pixels reduce distant-tag detail. A 960×540 or 1280×720 stream may crop or rescale: inspect actual calibrated FOV rather than assuming.

The Trench opening is nominally 22.25 in tall. A lens center at 22 in leaves only 0.25 in (6.35 mm) above it for the camera housing, bracket and dynamic clearance. If the robot must drive under the Trench, lower/protect the complete assembly and verify actual clearance; the 22 in optical suggestion is not mechanical approval.

Recommended prototype: lower camera 15° up, higher camera 10° up, one generally forward and one generally rearward, subject to clear views through the real mechanism. Do not lock in diagonal yaw offsets until checking how they affect both reversed orientations and lane positions. Full-sensor coverage helps but cannot restore tags outside horizontal view or behind solid objects.

## Test without a complete field

1. Use correctly scaled tags at center heights 0.552, 0.889 and 1.124 m. Preserve the surrounding border; verify the printed dimensions.
2. At each camera's real height/tilt, inspect complete-tag coverage and stable detections at practical distances such as 1, 2 and 4 m. Include horizontal offsets near both image edges.
3. Test the actual front/back mechanism views with intake and shooter in their normal operating positions, then reverse robot orientation. Do not infer visibility from the bare bracket alone.
4. Calibrate the selected camera/lens at the selected pipeline mode, and update measured robot-to-camera translations/rotations before using estimates. Check reported pose against measured stationary positions and headings, including when each camera sees a different structure.
5. Record detection rate, pose variation/error, ambiguity and latency while stationary, translating and rotating. Select a mode that maintains usable detail and processing rate; a sharp but stale measurement is not the goal.

## Reproduce

Python standard library only, from any computer:

```sh
python3 docs/vision/camera-mount-analysis/analyze.py --image-height 800
python3 docs/vision/camera-mount-analysis/analyze.py --image-height 720
```

Outputs: [full-frame samples](sample-visibility-800.csv), [cropped samples](sample-visibility-720.csv). No robot constants or operating camera settings were changed by this analysis.
