# Physics environment

The Simulation can move bodies by **forces** instead of prescribed motions. Bodies get mass, velocity,
charge and temperature. The world gets gravity, a floor, air, electric and magnetic fields, and
optionally special relativity. One engine covers:

| Area | What it does | Law |
| --- | --- | --- |
| **Gravity** | Uniform `g`, or Newton's gravitation between every pair of masses, or both | F = m g, F = G m₁ m₂ / r² |
| **Kinematics** | Position, velocity and acceleration of every body at any time | v = dx/dt, a = dv/dt |
| **Newton's laws** | Net force changes momentum; contacts push back equally and oppositely | F = dp/dt, F₁₂ = −F₂₁ |
| **Momentum** | Collisions exchange momentum through impulses; the total of a closed system stays constant | Σp = const |
| **Work & energy** | Kinetic, gravitational and electric energy, work done by applied forces | W = ∫F·dx, KE = ½mv² |
| **Conservation of energy** | Bounces and friction turn motion into heat; the air carries heat away; the total balances | KE + PE + Q + Q_air − W = const |
| **Electromagnetism** | Charges attract and repel; uniform E and B fields push charges | F = k q₁q₂/r², F = q(E + v×B) |
| **Waves & optics** | Sound, light or slow ripple sources: wavelength, expanding wavefronts, Doppler shift at every other body | λ = v/f, f' = f (v − v_o·n)/(v − v_s·n), relativistic Doppler for light |
| **Thermodynamics** | Body temperatures: heat from impacts and friction, conduction between touching bodies, cooling in air | Q = m c ΔT, Fourier, Newton's cooling |
| **Relativity** | Momentum γmv, so nothing reaches c; proper time; length contraction (lower c to see it) | γ = 1/√(1−v²/c²), dτ = dt/γ |
| **Quantum** | De Broglie wavelength and the uncertainty limit for every body (readouts) | λ = h/p, Δx Δp ≥ ħ/2 |

## Experiments

`PHYSICS DEMO name` (or **Motion Study → Physics environment → Experiment…**) loads one and frames it. Then
press Play.

| Demo | Shows |
| --- | --- |
| `drop` | Three balls fall 1 m with restitution 0.9 / 0.6 / 0.3. Each rebounds to e² of the height, and the lost kinetic energy becomes heat. |
| `projectile` | A ball thrown at 45°, 4.5 m/s, on a parabola: constant horizontal speed, a = −g vertically. Then it bounces and rolls on. |
| `collision` | Newton's cradle in space: the striker stops and the last ball leaves at its speed. Momentum and kinetic energy are conserved. |
| `orbit` | A moon on a circular orbit around a 10¹⁰ kg planet (Newton's gravitation, v = √(GM/r)). |
| `charges` | Opposite charges attract, like charges repel (Coulomb). |
| `cyclotron` | A charge in a 1 T field circles at r = mv/(qB). The magnetic force does no work. |
| `heat` | A 100 °C block on a 0 °C block: conduction to a common 50 °C, then slow cooling in the air. |
| `doppler` | A moving ripple source. Wavefronts bunch up ahead, so a listener in front hears a higher frequency. |
| `relativity` | c = 2 m/s. A pushed rocket approaches c but never reaches it. Its clock runs slow and it contracts. |

## How it works

- **One run, then playback.** The run is integrated once from t = 0 in SI units, in 0.5 ms steps, and
  cached. The timeline therefore plays, loops and scrubs exactly, and `PHYSICS AT t` or `SIM AT t` give
  the same answer headless. Changing anything (a mass, a field, a body's place) recomputes the run.
- **Integrator.** Velocity Verlet, which is symplectic, so orbits keep their energy. The magnetic force
  is applied as an exact rotation of the momentum (Boris), so a charge in a B field keeps its speed.
  Free fall and projectiles are exact.
- **Mass.** Volume × density from the body's mesh (default 1000 kg/m³), or a mass you set. Centre of
  mass from the mesh.
- **Collisions.** Momentum-conserving impulses with restitution, on exact spheres for balls and on the
  body's box otherwise (turned with it when it spins). The floor (z = 0) uses the exact mesh. A body
  resting on it is held by the floor's normal force.
  - **Boxes and other shapes** slide, with Coulomb friction.
  - **Balls** roll. An impact keeps 5/7 of the slip (a solid sphere starting to roll), and on the floor
    only rolling resistance (0.01) slows them.
  - Bodies don't tumble: spin is kept as set (no torques).
- **Energy accounting.** Everything a contact or friction takes out of the motion is added to the
  touching bodies as heat (their temperature rises by Q/(m c)). Drag and cooling go to the air. Applied
  forces add work. The readout shows the total and its drift from the start.
  - **Drift values:** 0.000 % for the drop, orbit, cyclotron, cradle, heat, Doppler and relativity demos.
    About 1 % for charges collapsing into contact under a strong attraction.
- **Fixed bodies** (planets, anchors, charged plates) do not move but still attract, repel and collide.
  If they have motions they follow them, so a moving paddle can hit a ball.
- **Waves.** A source emits a crest every period from wherever it is at that moment, so the drawn
  wavefronts show the Doppler effect. The readout gives the frequency heard at every other body.
  Sound uses 343 m/s, light uses c, and custom waves use your speed (ripples, slow motion).

## Commands

```text
PHYSICS ON | OFF                        (ON with no physics bodies yet adds every visible body)
PHYSICS DEMO [name]                     list, or load an experiment
PHYSICS AT t [body]                     report: world, momentum, energy and conservation, every body's
                                        kinematics, force, temperature, relativity, quantum and waves
PHYSICS GRAVITY OFF|UNIFORM|NEWTON|BOTH [g | x,y,z] [G value]     e.g. GRAVITY UNIFORM 1.62 (the Moon)
PHYSICS FLOOR ON|OFF [FRICTION mu]      PHYSICS AIR ON|OFF
PHYSICS FIELD E x,y,z | FIELD B x,y,z | FIELD OFF     (V/m, T)
PHYSICS AMBIENT degC                    PHYSICS DURATION s
PHYSICS RELATIVITY ON|OFF [C m/s]
PHYSICS BODY [name] DYNAMIC|FIXED|OFF [MASS kg] [DENSITY kg/m3] [VELOCITY x,y,z] [SPIN x,y,z]
             [CHARGE C] [TEMP degC] [HEAT J/kgK] [BOUNCE e] [FORCE x,y,z] [WAVE SOUND|LIGHT|CUSTOM|OFF]
             [FREQ Hz] [WAVESPEED v]
```

Velocities are lengths per second in the document's units (`VELOCITY 2m,0,1m` works in any unit).
Forces are in N, spin in deg/s and charge in C.

Example, a ball rolling off a table under air drag:

```text
BOX 800 600 750 @0,0,0          SPHERE 30 @-300,0,750
PHYSICS BODY "Block 1" FIXED    PHYSICS BODY "Sphere 2" VELOCITY 1.5m,0,0 BOUNCE 0.7 DENSITY 7850
PHYSICS AIR ON    SIM PLAY      PHYSICS AT 1.2 "Sphere 2"
```

## In the app

**Simulation → Motion Study → Physics environment:**

- **World settings:** gravity, g / G, floor and friction, air drag, relativity and c, E and B fields,
  ambient temperature, run length.
- **Selected body:** in the physics or not, fixed, mass, density, velocity, spin, applied force,
  charge (µC), temperature, specific heat, bounce, and waves with frequency and speed.
- **Readout:** a live readout at the playhead.

The 3D view draws the physics on top of the model:

- Velocity (green) and net force (orange) arrows.
- A temperature halo on each body: blue is cold, white is 20 °C, red is hot.
- Trails (*Paths*).
- E and B field arrows.
- Expanding wavefronts.
- Labels with speed, temperature and γ.
- An **energy balance** under the clock: kinetic, potential, heat, heat to the air and work done, with
  the conserved total, its drift, the momentum and the collision count.

*Fit* and *Home* frame everywhere the bodies go during the run.

Over MCP: `set_physics` (world, a body, or a demo) and `physics_report` (the readout at a time).
