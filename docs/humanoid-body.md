# Humanoid body library

![Humanoid parts](humanoid.png)

![March in place](humanoid-march.png)

Parametric humanoid robot parts for the **Library** panel (category *Humanoid body*) and the `PART`
command: head, neck, upper body (chest, heart, gut, midriff), low hip (pelvis), thighs, legs and feet.
Together with the [robot arm and hand](robot-arm.md) they make a complete **Humanoid Robot**, head to
toe. As with the arm and hand, every joint is a limited circular motion driven by the library **DC
motor** (used as a gear motor), so the robot moves in the Simulation window straight after it is
inserted.

## Human body vs robot body

| Human anatomy | Robot part (kind) | Joint(s) | Notes |
| --- | --- | --- | --- |
| Skull, face | **Head** (`robot_head`): rounded shell, dark **visor**, ear sensor pods | Neck pitch and roll at the head pivot | The visor stands in for the face / camera window |
| Cervical spine (C1–C7) | **Neck** (`robot_neck`): column with joint rings | Neck rotation at its base; nod and tilt at its top | Atlanto-occipital nod and atlanto-axial turn become 3 stacked revolute joints |
| Rib cage, shoulder girdle | **Chest** (`robot_chest`): thorax shell with shoulder caps | Carries both shoulders (see the arm) | The scapula / clavicle motion is left out |
| Heart | **Heart** (`robot_heart`): power core in the chest: housing, glowing core, coolant **pump impeller** | The pump turns continuously at the heart rate (72 rpm = 72 bpm by default) | Robots carry batteries and a coolant loop, not blood: the "heart" is the power and cooling core |
| Diaphragm / upper abdomen (midriff) | **Midriff** (`robot_midriff`): bellows boot over the lumbar actuator | Trunk flexion (bend forward) and lateral flexion at its base | Like the thoracolumbar spine |
| Abdomen (gut) | **Gut** (`robot_gut`): three armour plates over the dark **battery core** | Trunk rotation at its base (the waist) | The battery sits where the viscera are, low and central |
| Pelvis, hip sockets (low hip) | **Pelvis** (`robot_pelvis`): hip axle housings, waist bearing | Root of the body: the waist and both hips | The hip ball-and-socket joints (3 DOF each) become 3 stacked revolute joints |
| Femur (≈ 430 mm) | **Thigh** (`robot_thigh`): hip ball, tapered shell | Hip flexion, abduction, internal rotation | The knee motor sits on its back, like the hamstrings |
| Tibia + fibula (≈ 420 mm) | **Shin** (`robot_shin`): knee hub and knee cap, calf, ankle pin | Knee flexion | The ankle motors sit in the calf, like the calf muscles |
| Foot (tarsals, metatarsals) | **Foot** (`robot_foot`): ankle hub, rubber sole | Ankle plantarflexion and inversion | The subtalar and talocrural joints become 2 revolute joints |
| Toes (phalanges) | **Toe** (`robot_foot` part 1) with its own sole | Toe extension at the ball of the foot (MTP) | One hinged toe plate for all five toes |

The robot body has **20 joints** (neck 3, waist 1, trunk 2, each leg 7) plus **7 per arm** and the
hands (15 joints each), every one on its own DC gear motor.

## Joints and default range of motion

All angles are measured from the standing pose (0°), arms hanging. Each joint sweeps **0° → limit →
0°**; the parameter's maximum is the mechanical hard stop. Defaults follow normal human active range
of motion, rounded to robot-friendly values.

| Joint (`PART` key) | Motion from the standing pose | Default | Hard stop | Human ROM |
| --- | --- | --- | --- | --- |
| Neck rotation (`neck_yaw`) | turns the head to the left | 70° | 80° | 70–80° each way |
| Neck flexion (`neck_pitch`) | nods the chin down | 45° | 60° | 45–50° flexion, 60° extension |
| Neck lateral flexion (`neck_roll`) | tilts the head toward the right shoulder | 40° | 45° | 45° each way |
| Trunk rotation (`waist_rot`) | turns the upper body at the waist | 40° | 45° | 30–45° each way |
| Trunk flexion (`trunk_flex`) | bends forward at the midriff | 60° | 80° | 60° lumbar, 80° thoracolumbar |
| Trunk lateral flexion (`trunk_bend`) | bends sideways | 25° | 35° | 25–35° each way |
| Shoulder flexion (`shoulder_flex`) | swings the arm forward and up | 90° | 180° | 0–180° |
| Shoulder abduction (`shoulder_abd`) | raises the arm sideways | 90° | 180° | 0–180° |
| Shoulder internal rotation (`shoulder_rot`) | turns the upper arm inward | 70° | 90° | 70° internal / 90° external |
| Elbow, forearm, wrist, hand | as on the [robot arm](robot-arm.md) | | | |
| Hip flexion (`hip_flex`) | swings the thigh forward | 90° | 120° | 120° (knee bent), 90° straight leg |
| Hip abduction (`hip_abd`) | swings the leg out to the side | 40° | 45° | 45° |
| Hip internal rotation (`hip_rot`) | turns the toes inward | 35° | 45° | 40–45° |
| Knee flexion (`knee`) | brings the heel up behind | 130° | 140° | 135–140° |
| Ankle plantarflexion (`ankle_pitch`) | points the toes down | 45° | 50° | 50° plantar / 20° dorsiflexion |
| Ankle inversion (`ankle_roll`) | turns the sole inward | 30° | 35° | 35° inversion / 15° eversion |
| Toe extension (`toe`) | lifts the toes at the ball of the foot | 60° | 70° | 70° |

Extension past the standing pose (hyperextension, the other direction of the trunk and ankle) is not
modelled: the joints stop at 0°.

## Parts

| Part (`PART` key) | Bodies | Main parameters |
| --- | --- | --- |
| **Humanoid Robot** (`HUMANOID`) | Head, neck, chest, heart, midriff, gut, pelvis, 2 legs, 2 arms with hands, 20 + 14 + hand motors | `SIZE`, `FINISH` White / Black shells, `DEMO` Range of motion / March in place / Still, `DRIVE` (hands), `RPM`, `GEAR`, every limit above, `HEART_RATE` |
| **Robot Head & Neck** (`HEADNECK`) | Head, visor, neck on a pedestal, 3 motors | `SIZE`, `FINISH`, `RPM`, `GEAR`, `NECK_YAW` `NECK_PITCH` `NECK_ROLL` |
| **Robot Upper Body** (`TORSO`) | Pelvis on a stand, gut, midriff, chest, heart, 3 motors | `SIZE`, `FINISH`, `RPM`, `GEAR`, `WAIST_ROT` `TRUNK_FLEX` `TRUNK_BEND`, `HEART_RATE` |
| **Robot Leg** (`LEG`) | Hip mount on a stand, thigh, shin, foot, toe, 7 motors | `SIZE`, `FINISH`, `SIDE` Right / Left, `RPM`, `GEAR`, hip, knee, ankle and toe limits |
| **Robot Head** (`HEAD`) | Shell + visor | `WIDTH` `DEPTH` `HEIGHT` |
| **Robot Neck** (`NECK`) | 1 | `LENGTH` `RADIUS` |
| **Robot Chest** (`CHEST`) | 1 | `WIDTH` `DEPTH` `HEIGHT` |
| **Robot Heart (Power Core)** (`HEART`) | Housing, core, pump impeller (turning) | `RADIUS`, `HEART_RATE` |
| **Robot Gut (Abdomen)** (`GUT`) | Armour plates + battery core | `WIDTH` `DEPTH` `HEIGHT` |
| **Robot Midriff** (`MIDRIFF`) | Bellows | `WIDTH` `DEPTH` `HEIGHT` |
| **Robot Pelvis (Low Hip)** (`PELVIS`) | 1 | `WIDTH` `DEPTH` `HEIGHT` (waist above the hips) `HIP_SPACING` |
| **Robot Thigh** (`THIGH`) | 1 | `LENGTH` `WIDTH` `DEPTH` |
| **Robot Shin (Lower Leg)** (`SHIN`) | 1 | `LENGTH` `WIDTH` `DEPTH` |
| **Robot Foot** (`FOOT`) | Foot, toe, 2 rubber soles | `LENGTH` `WIDTH` `ANKLE` |

Single parts are set down on the ground at the insertion point; their kinds take the origin on the
joint they turn about (head pivot, neck base, waist, trunk joint, hip, knee, ankle), so they line up
when you build your own robot from them.

**Proportions (size 1, mm):** ankle 85 above the sole, shin 420, thigh 430 (hips 935 up, 190 apart),
waist 120 above the hips, gut 130, midriff 60, chest 290 (shoulder joints 400 apart, 1470 up), neck 80,
head 230: **1.83 m** overall. Arms: upper arm 300, forearm 250, so the fingertips hang at mid-thigh.

### Frame and design

- The robot stands on z = 0 **facing −Y**, toward the *Front* view; its right side is at −X (on your
  left in the Front view).
- Shells are swept rounded-box sections (like the hand): a domed head, a rib cage that widens to the
  shoulder caps, gut plates with the dark battery core showing between them, a bellows midriff, a
  pelvis with hip axle housings, tapered thighs, calves with a knee cap.
- **Finish:** white shells with dark joints (default) or black shells (Figure-style), in both cases
  with a dark glossy visor, a glowing cyan heart core and rubber soles.
- **Body orientation:** the arms are the library robot arm turned −90° about X, so they hang with the
  elbow bending forward and the palms facing back. Bodies now carry a `rotation` (degrees about X, then
  Y, then Z) for this; it is saved in the project and shown by `LIST`.

## How the DC motors drive the joints

As on the arm and hand ([details](robot-arm.md#how-the-dc-motors-drive-the-joints)): each joint has
its own DC motor named `<part> Motor <joint>` with a `… Drive` motion on its rotor. The drive oscillates
over `GEAR × limit` at `RPM`, and the joint follows at `1 / GEAR`, so it stops exactly at its limit.

Where the motors sit: waist and hip motors on the pelvis (hips on its sides), trunk motors on the back
of the gut, neck and shoulder motors on the upper back, the knee motor behind the thigh, the ankle motors
in the calf, the toe motor on the instep, the elbow motor on the upper arm and the forearm, wrist and
hand motors on the forearm, as on the arm.

Joints chain from the pelvis out: waist → trunk → neck → head and waist → trunk → shoulders → arm → hand;
hip → knee → ankle → toe. Every joint comes before every motor drive in the motion list, so a motor's
rotor turns in its housing however the body around it moves. (This also fixes the elbow and
forearm-rotation motor rotors of the robot arm, which drifted away from their housings when the arm
moved.)

## Demos

`DEMO` on the Humanoid Robot:

- **Range of motion** (default): every joint sweeps 0 → limit → 0 at once.
- **March in place:** the legs alternate (hip 50°, knee 75°, ankle 15°, 0.7 s per stride) and each arm
  swings with the opposite leg (shoulder 30°, elbow 30°); the other joints hold still.
- **Still:** only the heart pump turns; pose it yourself with `MOTION` or the Motion Study panel.

## Examples

```text
PART HUMANOID                                 white humanoid, every joint sweeping its range
PART HUMANOID FINISH 1 DEMO 1 @900,0,0        black humanoid marching in place
PART HUMANOID DEMO STILL KNEE 120 SIZE 0.8    smaller robot, standing still
PART HEADNECK NECK_YAW 60                     head and neck on a pedestal
PART TORSO HEART_RATE 90                      pelvis to chest, heart pump at 90 bpm
PART LEG SIDE LEFT @-700,0,0                  left leg on its stand
PART THIGH   PART SHIN   PART FOOT            single parts
SIM PLAY      SIM AT 0.7                       joint angles at 0.7 s
MOTION CIRCLE "Humanoid Robot 1 Motor Right Knee Drive" RANGE 240   knee to 60 deg at gear 4
```
