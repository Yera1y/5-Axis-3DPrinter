# DIY 4-/5-Axis FDM 3D Printer Research

**Research date:** 2026-11-12  
**Context:** Small desktop machine, approximately 200 mm build volume, self-designed in Shenzhen, target budget around US$500, with Klipper/CAD/Python experience.

## Table of contents

- [Executive summary](#executive-summary)
- [1. Architectures](#1-architectures)
- [2. Existing projects](#2-existing-projects)
- [3. Firmware and axis implementation](#3-firmware-and-axis-implementation)
- [4. Controller hardware](#4-controller-hardware)
- [5. Mechanical and electrical design](#5-mechanical-and-electrical-design)
- [6. Slicing and toolpath software](#6-slicing-and-toolpath-software)
- [7. Recommendation](#7-recommendation)
- [Sources](#sources)

## Executive summary

1. **VERIFIED:** For a first self-designed machine, a 4-axis rotating-tilted-nozzle (RTN) or polar Core R-Theta design is substantially more achievable than a general 5-axis printer ([Slicer4RTN](https://github.com/Spiritdude/Slicer4RTN), [Core R-Theta](https://github.com/jyjblrd/Core-R-Theta-4-Axis-Printer)).
2. **VERIFIED:** A 5-axis machine requires coordinated position/orientation kinematics and collision-aware toolpaths; ordinary planar slicers are insufficient ([LinuxCNC 5-axis kinematics](https://www.linuxcnc.org/docs/html/motion/5-axis-kinematics.html)).
3. **VERIFIED:** RepRapFirmware natively supports additional axes through `M584`, including `U V W A B C D`, and has documented rotary-axis homing examples ([Duet documentation](https://docs.duet3d.com/User_manual/Machine_configuration/Configuration_OpenPnP)).
4. **VERIFIED:** Klipper's official kinematics model is extensible, but standard Klipper does not provide a documented general-purpose B/C TCP printer implementation; custom kinematics are required ([Klipper kinematics](https://github.com/Klipper3d/klipper/blob/master/docs/Kinematics.md)).
5. **VERIFIED:** Marlin supports additional `A/B/C/U/V/W` axes and degrees for axes configured with `AXISn_ROTATES`, but this is not equivalent to full 5-axis TCP inverse kinematics ([Marlin stepper-driver documentation](https://marlinfw.org/docs/settings/stepper_drivers.html)).
6. **VERIFIED:** The most practical controller for six motors plus an extruder is a BTT Octopus/Octopus Pro, Manta M8P, Mellow FLY-Super8 Pro, or Duet 3 6HC ([BTT Octopus documentation](https://github.com/bigtreetech/docs/blob/master/docs/Octopus%20Pro.md), [Manta M8P documentation](https://github.com/bigtreetech/docs/blob/master/docs/M8P-V2_0.md)).
7. **VERIFIED:** Open5x, Core R-Theta, RotBot, Rep5x and Unlayered3D provide useful prior art, but none should be assumed to be a turnkey, production-ready DIY platform ([Open5x](https://github.com/FreddieHong19/Open5x), [Rep5x](https://github.com/dennisklappe/Rep5x)).
8. **INFERENCE:** The strongest portfolio path is a rigid Cartesian XYZ printer with a lightweight B-axis tilting nozzle and C-axis rotating bed, initially operated as ordinary XYZ.
9. **INFERENCE:** Under roughly US$500, structure, motors, electronics and a normal heated bed are feasible; precision rotary bearings, slip rings, custom hotend geometry and collision-aware software are the budget risks.
10. **RECOMMENDATION:** Build the 3-axis machine first, add one rotary axis, validate calibration/tool-center-point mathematics, then add the second rotary axis only after the 4-axis toolpath pipeline works.

**Evidence labels:** **VERIFIED** means directly supported by a primary repository, official documentation or paper. **INFERENCE** means engineering reasoning based on verified facts. **UNCERTAIN** means the claim could not be verified from accessible primary documentation. Source dates are included where relevant; many open-source examples are older than 2023 because the ecosystem remains small.

## 1. Architectures

The key distinction is whether the additional degrees of freedom move the tool, the workpiece, or both. LinuxCNC describes conventional 5-axis machines as three linear axes plus two rotary axes, with table-table, head-head and table-head configurations ([LinuxCNC](https://www.linuxcnc.org/docs/html/motion/5-axis-kinematics.html)).

| Architecture | DOF | Mechanical complexity / rigidity | Collision risk | Support-free potential | Real examples |
|---|---:|---|---|---|---|
| Cartesian XYZ + tilting nozzle + rotating bed, B/C | 5 | **VERIFIED/INFERENCE:** Two rotary joints add stiffness and calibration challenges; table-side axes preserve a relatively light tool. Table rotary/tilt configurations are standard 5-axis arrangements ([LinuxCNC](https://www.linuxcnc.org/docs/html/motion/5-axis-kinematics.html)). | High: nozzle, fan shroud and bed must be checked over orientation changes. | High for conformal surfaces and overhangs when the slicer chooses useful orientations. | Unlayered3D, Rep5x, IWK5X ([Rep5x](https://github.com/dennisklappe/Rep5x), [IWK5X](https://github.com/danielom123/IWK5X)). |
| Cartesian XYZ + tilting and rotating nozzle, B/C | 5 | **VERIFIED:** PAX uses two printhead rotations and firmware-level inverse kinematics; its author reports significant mechanical and software complexity ([XYZdims PAX](https://xyzdims.com/2021/02/08/3d-printing-penta-axis-pax-5-axis-printing-option/), **OLD**, 2021). | Very high because the complete printhead envelope rotates. | Very high in principle. | PAX, Open5x-style toolheads ([XYZdims PAX](https://xyzdims.com/2021/02/08/3d-printing-penta-axis-pax-5-axis-printing-option/)). |
| RTN: rotating tilted nozzle | 4 | **VERIFIED:** Usually one fixed nozzle tilt plus one rotation around Z/local cone axis; simpler than fully variable 5-axis orientation ([Slicer4RTN](https://github.com/Spiritdude/Slicer4RTN), **OLD**, 2021). | Moderate-to-high around nozzle and cooling duct. | Good for conical overhangs; not universal. | RotBot, Slicer4RTN, some Core R-Theta variants. |
| Cartesian XYZ + rotating bed | 4 | **INFERENCE:** Simpler than two rotary axes, but rotating mass reduces acceleration and introduces cable/heating problems. | Moderate; the part and bed rotate under a fixed tool. | Limited unless a specialized toolpath uses the rotary bed. | Rotary fourth-axis conversions and custom systems. |
| Polar R-Theta + Z + nozzle rotation | 4 | **VERIFIED:** Core R-Theta uses polar movement and a rotating nozzle; CAD, STEP/STL files, RRF configuration and BOM are published ([Core R-Theta repository](https://github.com/jyjblrd/Core-R-Theta-4-Axis-Printer)). | Moderate; rotating bed/nozzle changes the collision envelope. | Good for radial/conical parts; the project author describes its radial slicer as limited to simple parts ([Core R-Theta repository](https://github.com/jyjblrd/Core-R-Theta-4-Axis-Printer)). | Core R-Theta by Joshua Bird. |
| A/C trunnion: tilting and rotating bed | 5 | **VERIFIED:** A/C trunnion is a standard table-table configuration; IWK5X extends a Voron with an AC-trunnion bridge and publishes CAD/BOM/firmware ([LinuxCNC](https://www.linuxcnc.org/docs/html/motion/5-axis-kinematics.html), [IWK5X](https://github.com/danielom123/IWK5X)). | High: the part rotates into the nozzle and frame. | High for suitable surface orientations. | IWK5X; CNC-style trunnion designs. |
| Head-head 5-axis | 5 | **INFERENCE:** Highest moving-tool complexity and usually lowest desktop stiffness; long tool offsets amplify orientation errors. | Very high. | High. | PAX and some research machines. |
| Belt / infinite-Z printer | Usually 3 | **VERIFIED:** Belt printers use a continuously moving build surface and dedicated slicer/firmware profiles; they are not inherently 4-/5-axis systems ([BabyBeltPro](https://github.com/RobMink/BabyBeltPro)). | Moderate at belt/nozzle transition. | Not inherently improved; useful for long parts or batch production. | Blackbelt-style printers, BabyBeltPro. |
| Robot arm + extruder | Usually 6+ | **VERIFIED:** Robotic FFF supports continuous spatial paths and requires collision avoidance and coordinated robot kinematics ([MDPI robotic FFF paper](https://doi.org/10.3390/app11114825), **OLD**, 2021). | Very high: joint limits, singularities and self-collision matter. | Potentially very high. | Industrial robotic FFF and research systems. |
| Delta/SCARA/other parallel kinematics with rotary axes | 4–6 | **VERIFIED:** Klipper documents delta and other non-Cartesian kinematics, but not arbitrary TCP-oriented FDM ([Klipper kinematics](https://github.com/Klipper3d/klipper/blob/master/docs/Kinematics.md)). | High and geometry-dependent. | Potentially high, but software development burden is large. | Research prototypes; few accessible DIY examples. |

### Architecture assessment

**INFERENCE:** For a 200 mm desktop target, avoid starting with robot arms, head-head 5-axis designs, rotating heated beds with slip rings, or fully general A/B/C TCP motion. A Cartesian XYZ platform with a lightweight B tilt plus C rotation is the best balance between portfolio value and achievable calibration.

## 2. Existing projects

| Project | Architecture | Firmware/controller | BOM/documentation | Status/community | Known limitations |
|---|---|---|---|---|---|
| Open5x | Retrofit 3-axis printers to 5-axis printing; implementations include Prusa i3, Voron and E3D toolchanger systems ([repository](https://github.com/FreddieHong19/Open5x)). | Duet 2 configuration included; exact latest controller varies by implementation. | CAD, Duet configuration and Grasshopper definition; MIT licensed. | 1.2k stars, 179 forks, 213 commits at research date. | No single maintained hardware revision, standardized cost BOM or turnkey procedure was verified. |
| Core R-Theta | 4-axis polar R-Theta printer with rotating nozzle/axis ([repository](https://github.com/jyjblrd/Core-R-Theta-4-Axis-Printer)). | RepRapFirmware configuration included. | STEP/F3Z/STL files, PCB buildplate files, RRF configuration and BOM. | 945 stars, 117 forks, 28 commits. | Author says it is not plug-and-play and has no complete assembly instructions; radial slicer is limited. |
| Radial Non-Planar Slicer | Post-processes ordinary G-code for radial/polar printing ([repository](https://github.com/jyjblrd/Radial_Non_Planar_Slicer)). | Emits G-code with a rotary C axis in the notebook example. | Google Colab/Jupyter notebook, model and G-code folders. | 335 stars, 44 forks. | Intended for simple radial parts; users must change code for other 4-axis machines. |
| Unlayered3D / Unlayered Slicer | General non-planar and multidirectional FDM slicer ([repository](https://github.com/Unlayered3D/Unlayered-Slicer)). | Associated with a custom Unlayered3D RepRapFirmware fork ([fork](https://github.com/Unlayered3D/RepRapFirmware)). | AGPL-3.0; active C++ project. | 41 stars, 11 forks for slicer; custom RRF fork has 2 stars and 3 forks. | Broad hardware validation, stable releases and complete TCP behavior for arbitrary DIY machines were not verified. |
| Rep5x | Retrofit adds yaw/C and tilt/B to standard XYZ ([repository](https://github.com/dennisklappe/Rep5x)). | Marlin firmware configuration builder. | Build guide, STL/CAD-related files, BOM, wiring and printer-specific guides. | 50 stars, 0 forks; Ender 5 Pro and Ender 3 V3 SE documented. | Generic machines have guidelines only; project is explicitly beta. |
| RotBot Transform | 4-axis conical printing with tilted nozzle ([repository](https://github.com/RotBotSlicer/Transform)). | External G-code transformation rather than dedicated firmware. | GPL-3.0; transforms STL, runs a conventional slicer, back-transforms G-code. | 265 stars, 53 forks, 31 commits; main paper **OLD**, 2021. | No general collision checker; geometry transformation can fail for complex geometry. |
| Slicer4RTN | 4-axis RTN and experimental 5-axis modes ([repository](https://github.com/Spiritdude/Slicer4RTN)). | Configurable axis letters, default A for rotation and B for tilt. | LGPL-3.0; command-line Perl tool using Cura/Slic3r/PrusaSlicer-derived workflows. | 78 stars, 11 forks; version 0.6.0 documentation is **OLD**. | Explicitly experimental; one conic mode per model and no multiple conic centers. |
| PAX | Variable rotating and tilting nozzle, 4- or 5-axis ([XYZdims](https://xyzdims.com/2021/02/08/3d-printing-penta-axis-pax-5-axis-printing-option/)). | Author reports inverse kinematics implemented in firmware. | Mechanical concepts and photos documented, but no complete standardized build repository was verified. | Individual project/write-up; no reliable community-size metric. | Heavy printhead, collision detection, cooling geometry and software complexity. |
| IWK5X | Voron CoreXY plus A/C trunnion bridge; Ø220 × 195 mm stated build volume ([repository](https://github.com/danielom123/IWK5X)). | RepRapFirmware; Duet3 WiFi plus DuetX5 expansion stated by project. | Complete design files, BOM and firmware configuration claimed. | Recent open-source research project; community size not established. | Long-term community validation and reproducibility are not yet clear. |

## 3. Firmware and axis implementation

| Firmware | Extra axes | Rotary units/wrapping | TCP/inverse kinematics | Extruder, shaping, pressure advance | Suitability |
|---|---|---|---|---|---|
| RepRapFirmware | `M584` maps drives to `U V W A B C D`; Duet 3 supports many additional axis letters depending on board/firmware ([motor configuration](https://docs.duet3d.com/User_manual/Connecting_hardware/Motors_configuring)). | Standard configuration supports rotary axes as ordinary axes, but continuous wrapping is not clearly established in stable official documentation. A continuous-rotation request exists ([issue #780](https://github.com/Duet3D/RepRapFirmware/issues/780)). | Custom machine kinematics are supported; an “Adding New Kinematics” developer document exists ([fork](https://github.com/Unlayered3D/RepRapFirmware/blob/main/AddingNewKinematics.md)). | Extruders are separate E drives. Input shaping supports CAN expansion boards from RRF 3.5 onward ([input shaping](https://docs.duet3d.com/User_manual/Tuning/Input_shaping)). | Best documented choice for a custom B/C machine. |
| Klipper | Official config supports standard kinematics and generic Cartesian carriages, but not a documented arbitrary A/B/C printer model ([kinematics](https://github.com/Klipper3d/klipper/blob/master/docs/Kinematics.md), [generic Cartesian example](https://github.com/Klipper3d/klipper/blob/master/config/example-generic-caretesian.cfg)). | No official general rotary-axis wrapping/TCP workflow was found; custom forks can add behavior. | Kinematics classes convert Cartesian motion to stepper positions; arbitrary kinematics require Python/C implementation. | Pressure Advance and resonance compensation are mature features ([Pressure Advance](https://www.klipper3d.org/Pressure_Advance.html), [resonance compensation](https://www.klipper3d.org/Resonance_Compensation.html)). | Excellent if writing custom kinematics; not the lowest-risk first implementation. |
| Marlin 2.x | Additional internal axes `I/J/K/U/V/W` can be exposed as `A/B/C/U/V/W`; additional drivers/endstops are configured in firmware ([stepper drivers](https://marlinfw.org/docs/settings/stepper_drivers.html)). | `AXISn_ROTATES` expresses rotary coordinates in degrees; `M92` supports steps per degree ([M92](https://marlinfw.org/docs/gcode/M092.html)). | Native arbitrary TCP inverse kinematics in standard Marlin was not verified. | E0–E7 are documented; Linear Advance and input shaping are available ([Linear Advance](https://marlinfw.org/docs/features/lin_advance.html), [Input Shaping](https://marlinfw.org/docs/features/input_shaping.html)). | Good for Rep5x-style direct rotary axes; less attractive for general 5-axis TCP. |
| grblHAL | Supports multiple axes depending on driver/MCU; a documented issue response states Teensy 4.1 can support up to five axes ([axis issue](https://github.com/grblHAL/core/issues/333)). | Rotary axes can be configured as unitless/step-per-degree and have soft limits disabled for infinite rotation ([issue](https://github.com/grblHAL/core/issues/333)). | CNC-oriented; no standard FDM TCP/extrusion workflow was verified. | No native FDM pressure advance comparable to Klipper/Marlin was verified. | Useful for CNC-style motion experiments, not recommended for mature extrusion control. |
| FluidNC | Documentation describes up to 12 motors on six axes and axis names `X/Y/Z/A/B/C` ([axes](http://wiki.fluidnc.com/en/config/axes)). | `A/B/C` are traditionally rotary and do not report inches like XYZ; exact continuous rotation depends on limits/configuration. | No general TCP FDM implementation was verified. | CNC-oriented; no native FDM pressure-advance workflow was verified. | Suitable for inexpensive motion-control experiments. |
| LinuxCNC | Arbitrary joints/axes and custom kinematics components; official XYZAC and XYZBC table-rotary examples ([5-axis kinematics](https://www.linuxcnc.org/docs/html/motion/5-axis-kinematics.html)). | Rotary axes use CNC G-code conventions, including inverse-time `G93` and units-per-minute `G94` ([G-code reference](https://github.com/LinuxCNC/linuxcnc/blob/master/docs/src/gcode/g-code.adoc)). | Best verified TCP/IK capability: explicit forward/inverse transforms, pivot offsets and custom kinematics. | FDM extrusion, pressure advance and input shaping are less turnkey than Klipper/Marlin. | Best for a research-grade kinematics demonstrator, overkill for low-cost desktop FDM. |

### The important implementation distinction

A rotary-axis definition is not the same as TCP. A command such as:

```gcode
G1 X50 Y50 Z20 B15 C90
```

works correctly only if firmware performs the inverse transformation, a post-processor converts desired tool pose into machine coordinates, or the slicer directly emits machine-coordinate B/C motion. LinuxCNC distinguishes tool-tip position/orientation from joint coordinates and derives inverse transformations using homogeneous matrices ([LinuxCNC](https://www.linuxcnc.org/docs/html/motion/5-axis-kinematics.html)).

For a nozzle-tip offset vector `p` from the rotary pivot:

```text
tool_tip_position = pivot_position + R(B,C) · p
```

where `R(B,C)` is the combined rotation matrix. **VERIFIED:** LinuxCNC documents this class of transformation and pivot offset handling ([LinuxCNC](https://www.linuxcnc.org/docs/html/motion/5-axis-kinematics.html)). **INFERENCE:** A custom Klipper or Marlin implementation must either implement the transform in firmware, post-process XYZ toolpaths into machine XYZBC, or use a slicer that natively understands the machine.

## 4. Controller hardware

Assume one motor each for X, Y, Z, B, C and E: six stepper channels minimum. Reserve additional channels if Z uses dual motors or the extruder is dual-drive.

| Board | Drivers | Driver type | Host requirement | Availability/price evidence | Assessment |
|---|---:|---|---|---|---|
| Duet 3 Mini 5+ | 5 onboard; expansion can add two drivers or CAN devices ([official specs](https://www.duet3d.com/duet3mini5plus?manufacturer_id=11)). | TMC2209, up to 1.4 A RMS / 2 A peak stated by manufacturer. | Optional SBC; onboard networking available. | **VERIFIED price snapshot:** US$159.99 from a reseller ([KB-3D](https://kb-3d.com/store/duet3d/888-duet-3-mini-5-main-controller-board-multiple-types-5060662920479.html)). | Good RRF board, but five onboard drivers are insufficient for XYZBC+E without expansion. |
| Duet 3 6HC | 6 onboard high-current drivers ([official specs](https://beta.duet3d.com/duet3mainboard6hc)). | TMC2160A, up to 6.3 A peak / 4.45 A RMS. | Network; optional SBC/CAN expansion. | **VERIFIED price snapshot:** about US$329.99 ([Filastruder](https://www.filastruder.com/products/duet-3-mainboard-6hc)). | Directly fits six motors but consumes most of a US$500 project budget. |
| BTT Octopus/Octopus Pro | Up to eight drivers, nine stepper outputs in Pro documentation ([BTT docs](https://github.com/bigtreetech/docs/blob/master/docs/Octopus%20Pro.md)). | TMC2209, TMC5160 and other plug-in drivers; Pro supports motor voltage up to 60 V. | Raspberry Pi/host for Klipper; Marlin/RRF can run on MCU. | **VERIFIED price snapshot:** about US$62.99 ([West3D](https://west3d.com/products/btt-48v-octopus-pro-controller-board-3d-printer-control-system)). BTT is Shenzhen-based. | Best value for a custom six-axis machine. |
| BTT Manta M8P | Designed for eight stepper drivers ([BTT M8P docs](https://github.com/bigtreetech/docs/blob/master/docs/M8P-V2_0.md)). | TMC2209/TMC5160-class plug-ins depending model. | Can mount CB1/CM4 directly, reducing wiring. | Chinese manufacturer and official documentation; current primary-store price not verified. | Excellent Klipper platform if a CB1/CM4 or Raspberry Pi is available. |
| BTT SKR 3/SKR 3 EZ | Multiple stepper outputs; exact usable channels depend on variant ([SKR repository](https://github.com/bigtreetech/SKR-3/blob/master/README.md)). | TMC2209/TMC5160/EZ driver ecosystem. | Host required for Klipper; RRF support listed by BTT. | China-sourceable; current price not verified here. | Viable, but confirm exact driver count and pin mappings. |
| Mellow FLY-Super8 Pro | 8 driver ports ([pinout](https://mellow.klipper.cn/en/docs/ProductDoc/MainBoard/fly-super/fly-super8-pro/pin/)). | TMC2209/TMC5160-class configurations documented ([sample configuration](https://mellow-3d.github.io/fly_super8_pro_h723_klipper_mcu_config.html)). | Raspberry Pi/host for Klipper. | China-sourceable; current primary-store price not verified. | Strong Klipper option with Chinese documentation. |
| CAN toolboard | Adds local extruder/heater/fan/stepper control. | CAN-FD/CAN depending ecosystem. | Mainboard or host must support CAN. | Duet, BTT and Mellow provide CAN options ([Duet CAN-FD](https://docs.duet3d.com/User_manual/Connecting_hardware/CAN-FD_bus), [BTT Octopus](https://github.com/bigtreetech/docs/blob/master/docs/Octopus%20Pro.md)). | Useful for rotating/tilting toolheads, but adds communication and cable-management complexity. |

### Controller recommendation

**INFERENCE:** Use a BTT Octopus Pro plus Raspberry Pi/CB1 and Klipper for maximum freedom and low cost; use BTT Octopus/Manta plus Marlin for simpler direct rotary-axis experiments; use Duet 3 6HC plus RepRapFirmware when firmware-native additional axes and existing RRF examples matter more than budget.

## 5. Mechanical and electrical design

### Tilt mechanisms

| Mechanism | Assessment |
|---|---|
| Worm gear | **INFERENCE:** High reduction and self-locking can resist nozzle torque, but backlash and poor efficiency are risks. |
| Timing-belt reduction | Cheap and easy to source; backlash can be low with tension, but belt elasticity affects orientation accuracy. |
| Harmonic drive | Compact and low-backlash in principle, but commercial units may exceed the budget; DIY strain-wave gears require precision. |
| Direct-drive stepper | Simplest mathematically, but more sensitive to nozzle collision and has lower output torque. |
| Worm plus encoder | Potentially best collision recovery/repeatability, but adds cost and control complexity. |

PAX documentation explicitly identifies mechanical complexity and collision detection as major difficulties when the printhead can rotate freely ([XYZdims PAX](https://xyzdims.com/2021/02/08/3d-printing-penta-axis-pax-5-axis-printing-option/)).

### Rotating bed

**VERIFIED:** Conventional A/C table arrangements require coordinate transforms that include pivot and axis offsets ([LinuxCNC](https://www.linuxcnc.org/docs/html/motion/5-axis-kinematics.html)).

**INFERENCE:** Use a large-diameter crossed-roller or angular-contact bearing if budget permits. A lazy-Susan bearing is cheap but typically has excessive runout for a precision printer. A toothed belt gives low cost and easy ratio changes; a gear ring is compact but introduces backlash unless preloaded. Put the rotary axis close to the build-plane center and keep the rotating assembly light.

### Heated rotating bed

**UNCERTAIN:** No primary DIY source was found establishing a universally reliable low-cost heated rotary-bed design.

Options are a stationary heated bed, a slip ring, wireless power transfer, or a stationary heater under a rotating thin plate. **INFERENCE:** For the first machine, use a stationary heated bed and put rotation on the nozzle or on a cold unheated workholding table.

### Motors, homing and wiring

- **VERIFIED:** RRF and Marlin support separate rotary-axis configuration and homing examples ([Duet OpenPnP configuration](https://docs.duet3d.com/User_manual/Machine_configuration/Configuration_OpenPnP), [Marlin M92](https://marlinfw.org/docs/gcode/M092.html)).
- **INFERENCE:** Use 0.9° NEMA17 motors for B/C if the rotating assembly is light; use 1.8° NEMA17 or NEMA23 only when torque calculations justify the mass.
- **INFERENCE:** Home rotary axes with a repeatable mechanical index, optical interrupter, Hall sensor or hard-stop plus slow approach.
- **INFERENCE:** Do not rely on sensorless homing for a high-value orientation datum until repeatability is measured.
- **VERIFIED:** Rep5x uses slip-ring integration to prevent cable tangling ([Rep5x](https://github.com/dennisklappe/Rep5x)).

### Hotend geometry

PAX documentation reports that cooling-shroud and printhead geometry become important when the tool tilts ([XYZdims PAX](https://xyzdims.com/2021/02/08/3d-printing-penta-axis-pax-5-axis-printing-option/)).

**INFERENCE:** Use a short, narrow fan duct; put the B/C pivot close to the nozzle tip; measure the nozzle-tip offset in CAD and calibrate physically; design a sacrificial collision bracket or mechanical clutch; keep filament routing above rotary axes with a controlled bend radius.

## 6. Slicing and toolpath software

| Tool | Architectures | Collision checking | TCP/coordinate conversion | G-code/maturity | Licence/assessment |
|---|---|---|---|---|---|
| Unlayered Slicer | General non-planar and multidirectional printing; exact supported machine geometries are project-dependent ([repository](https://github.com/Unlayered3D/Unlayered-Slicer)). | **UNCERTAIN:** Complete public collision-checking specification was not verified. | Associated with custom RRF work; exact division between slicer and firmware is not fully public. | Active C++ project with 41 stars/11 forks. | AGPL-3.0; promising but immature for a first machine. |
| Open5x Cura + Grasshopper | Open5x retrofit architecture; Grasshopper definition included ([Open5x](https://github.com/FreddieHong19/Open5x)). | **UNCERTAIN:** Complete collision-checking specification not found. | Grasshopper definition and Duet configuration provide project-specific transformation. | Open5x is MIT licensed; research-oriented. | Good for learning and portfolio documentation. |
| Radial Non-Planar Slicer | Core R-Theta and similar polar/4-axis machines ([repository](https://github.com/jyjblrd/Radial_Non_Planar_Slicer)). | Limited; geometric conversion, not general collision planning. | Notebook converts Cartesian positions to polar coordinates and emits rotary C with nozzle-offset correction ([notebook](https://github.com/jyjblrd/Radial_Non_Planar_Slicer/blob/main/main.ipynb)). | Python/Jupyter/Colab; useful prototype. | Best starting point for Core R-Theta-like machines. |
| RotBot Transform | RotBot and variable-angle conical printing ([repository](https://github.com/RotBotSlicer/Transform)). | No general collision checking documented. | STL pre-transformation, conventional slicing, G-code back-transformation. | Python, GPL-3.0, experimental but usable; main paper **OLD**, 2021. | Excellent educational fallback for fixed-angle RTN. |
| Slicer4RTN | 4-axis RTN; experimental 5-axis output ([repository](https://github.com/Spiritdude/Slicer4RTN)). | No complete collision checker; explicitly experimental. | Configurable A/B output; unlimited or single-revolution modes documented. | Perl CLI, LGPL-3.0, version 0.6.0 documentation. | Useful fallback, not unattended production software. |
| FullControl | Arbitrary custom toolpaths ([repository](https://github.com/FullControlXYZ/fullcontrol)). | User-designed paths; no general automatic collision checking claimed. | Python directly designs/exports G-code; experimental `lab/fullcontrol/multiaxis` exists but is not hardware-adaptable by default ([README](https://github.com/FullControlXYZ/fullcontrol/blob/master/README.md)). | GPL-3.0; 1k stars/128 forks. | Best for custom research paths and test coupons. |
| Rhino/Grasshopper/Kangaroo | Any architecture for which geometry/postprocessor are implemented. | Can be implemented with mesh intersection and simulation, but not automatic by default. | User-written transforms/postprocessor. | No single standard 5-axis FDM workflow was verified. | Excellent for explaining mathematics visually. |
| PrusaSlicer/OrcaSlicer non-planar modifications | Mostly planar/non-planar experimental workflows; rotary support varies by fork. | Usually limited to mesh/path checks, not complete moving-tool collision simulation. | Usually post-processing or custom fork. | A current mainstream release with general B/C TCP output was not verified. | Use ordinary slicers for initial XYZ mode. |
| Research slicers | Surface-following, decomposition and support-free multi-axis printing. | Papers often include orientation/accessibility/collision constraints, but implementations vary. | Usually postprocessor or research-specific machine model. | Recent papers describe decomposition and five-axis support-free workflows ([MDPI 2025](https://www.mdpi.com/2076-0825/14/2/99), [Springer 2025](https://link.springer.com/article/10.1007/s12206-025-1044-1)). | Useful references, not necessarily usable software. |

### Recommended software path

1. Use OrcaSlicer, PrusaSlicer or Cura for normal XYZ.
2. Write a Python postprocessor that accepts ordinary XYZ G-code and emits one rotary axis.
3. Add nozzle-offset compensation and verify it against CAD.
4. Add collision checking using the hotend mesh and swept-volume tests.
5. Move inverse kinematics into firmware only if a clean machine-level interface requires it.

## 7. Recommendation

### Ranked architecture recommendation

#### 1. Cartesian XYZ + B tilting nozzle + C rotating workpiece

**INFERENCE:** Best final architecture for your goals. It preserves ordinary XYZ operation, makes an impressive 5-axis portfolio project, supports stationary-bed development, lets B/C motion be tested independently, and provides a clear homogeneous-transform/TCP project.

Recommended initial axis arrangement:

```text
X/Y/Z: conventional Cartesian
B: nozzle tilt about X or Y
C: nozzle/workpiece rotation about Z
E: direct-drive extruder
```

Start with one active rotary axis. Add the second after calibration.

#### 2. Core R-Theta 4-axis

**INFERENCE:** Best if the priority is novel mechanical design with a lower software target. It has a proven open-source reference, published CAD/firmware/BOM, fewer coordinated axes and a strong connection to polar-coordinate mathematics. Its disadvantages are an unusual build envelope, radial-slicer limitations and poor compatibility with ordinary slicers.

#### 3. Fixed-angle RTN / RotBot

**VERIFIED:** Available software is documented but experimental and limited to conic modes ([RotBot](https://github.com/RotBotSlicer/Transform), [Slicer4RTN](https://github.com/Spiritdude/Slicer4RTN)). This is the lowest-risk route for demonstrating support reduction without implementing a full 5-axis slicer.

#### 4. A/C trunnion bed

**INFERENCE:** Excellent mechanically and academically, but harder to keep small, rigid and inexpensive. IWK5X is an important reference ([IWK5X](https://github.com/danielom123/IWK5X)).

### Recommended electronics

Use a BTT Octopus Pro, Manta M8P or Mellow FLY-Super8 Pro with Klipper if low cost and custom software are priorities. Use Duet 3 6HC with RepRapFirmware if existing RRF axis configuration and firmware-side kinematics are more important than budget.

### Approximate BOM

The following is a planning estimate, not a verified quotation. Verify current Taobao/AliExpress/JLCPCB prices.

| Item | Suggested part | Estimated cost |
|---|---|---:|
| Controller | BTT Octopus Pro or Manta M8P | UNCERTAIN: US$50–100 |
| Host | Raspberry Pi, CB1 or existing Linux SBC | UNCERTAIN: US$0–80 |
| Stepper motors | 5–6 NEMA17 | UNCERTAIN: US$40–80 |
| Drivers | TMC2209 or TMC5160 modules if not onboard | UNCERTAIN: US$15–60 |
| Linear motion | MGN12/MGN15 rails, lead screw, belts | UNCERTAIN: US$70–120 |
| Frame | 2020/2040 aluminium extrusion | UNCERTAIN: US$40–70 |
| Heated bed | 200 mm silicone heater, aluminium plate, SSR/MOSFET | UNCERTAIN: US$30–60 |
| Hotend/extruder | V6/CHC-style hotend and direct-drive extruder | UNCERTAIN: US$30–70 |
| Rotary mechanisms | Bearings, belts/gears, couplers, shafts | UNCERTAIN: US$50–120 |
| Endstops/sensors | Switches, Hall/optical index sensors | UNCERTAIN: US$10–30 |
| Wiring/safety | Connectors, ferrules, drag chains, fuses, enclosure | UNCERTAIN: US$30–60 |
| Custom parts | JLCPCB brackets/PCBs, printed parts | UNCERTAIN: US$20–80 |
| **Total** |  | **UNCERTAIN: approximately US$385–890** |

**INFERENCE:** Staying below US$500 is realistic only if you reuse an existing host, source locally, avoid a premium Duet board, and delay expensive precision rotary hardware.

### Staged build plan

#### Stage 0 — Mathematical simulation

Implement forward kinematics, inverse kinematics, nozzle-tip offset, B/C rotation order, rotary-axis wrapping, workspace/singularity checks and simple collision-envelope tests in Python. Use LinuxCNC's transformation discussion as the mathematical reference ([LinuxCNC](https://www.linuxcnc.org/docs/html/motion/5-axis-kinematics.html)).

#### Stage 1 — Conventional 200 mm XYZ machine

Build and tune reliable ordinary prints. Validate frame squareness, steps/mm, thermal behavior, extrusion calibration, input shaping and pressure advance. Klipper documents pressure advance and resonance compensation ([Pressure Advance](https://www.klipper3d.org/Pressure_Advance.html), [Resonance Compensation](https://www.klipper3d.org/Resonance_Compensation.html)).

#### Stage 2 — Rotary-axis bench test

Build only B or C. Measure angular repeatability, backlash, static holding torque, missed steps, homing repeatability and cable drag. Do not install a hotend yet.

#### Stage 3 — Tool-center-point test

Mount a pointer or dial indicator instead of a heated nozzle. Command B0, B15, B30 and C0, C90; compare measured tip displacement with the CAD transform.

#### Stage 4 — Cold extrusion

Use PLA at low speed with normal XYZ paths, B-only motion, C-only motion, and combined XYZ+B/C motion without support-free geometry.

#### Stage 5 — 4-axis conic printing

Use RotBot or Slicer4RTN as a baseline, then compare your own Python postprocessor against it ([RotBot](https://github.com/RotBotSlicer/Transform), [Slicer4RTN](https://github.com/Spiritdude/Slicer4RTN)).

#### Stage 6 — 5-axis support-free experiments

Start with tilted planes, cylindrical shells, overhang wedges, curved ramps and stress-aligned coupons. Do not begin with arbitrary organic models. General support-free printing requires orientation planning, segmentation, collision checking and path continuity ([MDPI 2025](https://www.mdpi.com/2076-0825/14/2/99)).

### Biggest risks and mitigations

| Risk | Why it matters | Mitigation |
|---|---|---|
| Backlash | Rotary backlash becomes nozzle-position error. | Use belt reduction or preloaded gearing; measure reversal error. |
| Nozzle-offset calibration | Angular error produces tip displacement proportional to pivot distance. | Put the pivot close to the nozzle; calibrate with a dial indicator. |
| Collision | Rotating ducts/hotends can strike the part or frame. | Model the tool envelope; run swept-volume checks. |
| Firmware mismatch | Axis labels do not provide TCP. | Decide whether firmware, postprocessor or slicer owns IK. |
| Rotary wrapping | A path may choose 359° instead of −1°. | Implement continuous-angle unwrapping and test it separately. |
| Cable fatigue | Rotating toolheads twist wires and filament paths. | Use a controlled cable loop; use a slip ring only when necessary. |
| Heated rotating bed | Power transfer and thermal uniformity add risk. | Keep the bed stationary in the first machine. |
| Extrusion synchronization | Orientation changes alter path length and speed. | Calculate extrusion from actual TCP path length; validate thin walls. |
| Software scope | General 5-axis slicing is much harder than motion control. | Start with fixed-angle/conic paths and scripted FullControl-style paths. |
| Budget overrun | Precision bearings, encoders and premium controllers dominate cost. | Use an Octopus/Manta/Mellow board; postpone encoders/slip rings. |

### Unverified points

- **UNCERTAIN:** A mainstream Klipper release with native arbitrary 5-axis FDM TCP kinematics.
- **UNCERTAIN:** Stable, general continuous-rotation wrapping semantics in stock RepRapFirmware.
- **UNCERTAIN:** Current Chinese retail prices and clone quality for every controller.
- **UNCERTAIN:** A complete collision-checking implementation in Unlayered Slicer.
- **UNCERTAIN:** A turnkey, actively maintained PAX hardware BOM.
- **UNCERTAIN:** Community-size metrics beyond GitHub stars/forks.
- **UNCERTAIN:** A current PrusaSlicer or OrcaSlicer release with general B/C TCP output.
- **UNCERTAIN:** A universally reliable low-cost heated rotating-bed design using slip rings or wireless power.
- **UNCERTAIN:** Production-quality support-free slicing for arbitrary models across all listed architectures.

## Sources

### Projects and slicers

- Open5x: https://github.com/FreddieHong19/Open5x
- Core R-Theta 4-Axis Printer: https://github.com/jyjblrd/Core-R-Theta-4-Axis-Printer
- Radial Non-Planar Slicer: https://github.com/jyjblrd/Radial_Non_Planar_Slicer
- Radial Non-Planar Slicer notebook: https://github.com/jyjblrd/Radial_Non_Planar_Slicer/blob/main/main.ipynb
- Unlayered Slicer: https://github.com/Unlayered3D/Unlayered-Slicer
- Unlayered RepRapFirmware fork: https://github.com/Unlayered3D/RepRapFirmware
- Rep5x: https://github.com/dennisklappe/Rep5x
- RotBot Transform: https://github.com/RotBotSlicer/Transform
- Slicer4RTN: https://github.com/Spiritdude/Slicer4RTN
- FullControl: https://github.com/FullControlXYZ/fullcontrol
- IWK5X: https://github.com/danielom123/IWK5X
- BabyBeltPro: https://github.com/RobMink/BabyBeltPro
- PAX write-up: https://xyzdims.com/2021/02/08/3d-printing-penta-axis-pax-5-axis-printing-option/
- PAX sub-volume/non-planar slicing: https://xyzdims.com/2021/04/19/3d-printing-sub-volume-segmenting-non-planar-slicing/

### Firmware and kinematics

- Klipper kinematics: https://github.com/Klipper3d/klipper/blob/master/docs/Kinematics.md
- Klipper generic Cartesian example: https://github.com/Klipper3d/klipper/blob/master/config/example-generic-caretesian.cfg
- Klipper pressure advance: https://www.klipper3d.org/Pressure_Advance.html
- Klipper resonance compensation: https://www.klipper3d.org/Resonance_Compensation.html
- RepRapFirmware motor configuration: https://docs.duet3d.com/User_manual/Connecting_hardware/Motors_configuring
- RepRapFirmware OpenPnP axis example: https://docs.duet3d.com/User_manual/Machine_configuration/Configuration_OpenPnP
- RepRapFirmware input shaping: https://docs.duet3d.com/User_manual/Tuning/Input_shaping
- RepRapFirmware continuous-rotation issue: https://github.com/Duet3D/RepRapFirmware/issues/780
- Unlayered RepRapFirmware kinematics development: https://github.com/Unlayered3D/RepRapFirmware/blob/main/AddingNewKinematics.md
- Marlin stepper drivers and additional axes: https://marlinfw.org/docs/settings/stepper_drivers.html
- Marlin M92 axis units: https://marlinfw.org/docs/gcode/M092.html
- Marlin Linear Advance: https://marlinfw.org/docs/features/lin_advance.html
- Marlin Input Shaping: https://marlinfw.org/docs/features/input_shaping.html
- grblHAL rotary-axis issue: https://github.com/grblHAL/core/issues/333
- FluidNC axes: http://wiki.fluidnc.com/en/config/axes
- LinuxCNC 5-axis kinematics: https://www.linuxcnc.org/docs/html/motion/5-axis-kinematics.html
- LinuxCNC G-code reference: https://github.com/LinuxCNC/linuxcnc/blob/master/docs/src/gcode/g-code.adoc

### Controller hardware

- Duet 3 Mini 5+ official product page: https://www.duet3d.com/duet3mini5plus?manufacturer_id=11
- Duet 3 Mini 5+ hardware documentation: http://docs.duet3d.com/Duet3D_hardware/Duet_3_family/Duet_3_Mini_5+_Hardware_Overview
- Duet 3 6HC official product page: https://beta.duet3d.com/duet3mainboard6hc
- Duet 3 6HC documentation: http://docs.duet3d.com/Duet3D_hardware/Duet_3_family/Duet_3_Mainboard_6HC_Hardware_Overview
- Duet 3 Mini 5+ reseller price: https://kb-3d.com/store/duet3d/888-duet-3-mini-5-main-controller-board-multiple-types-5060662920479.html
- Duet 3 6HC reseller price: https://www.filastruder.com/products/duet-3-mainboard-6hc
- BTT Octopus Pro documentation: https://github.com/bigtreetech/docs/blob/master/docs/Octopus%20Pro.md
- BTT Octopus Pro reseller price: https://west3d.com/products/btt-48v-octopus-pro-controller-board-3d-printer-control-system
- BTT Manta M8P documentation: https://github.com/bigtreetech/docs/blob/master/docs/M8P-V2_0.md
- BTT SKR 3 repository: https://github.com/bigtreetech/SKR-3/blob/master/README.md
- BTT SKR 3 EZ documentation: https://github.com/bigtreetech/docs/blob/master/docs/SKR%203%20EZ.md
- Mellow FLY-Super8 Pro pinout: https://mellow.klipper.cn/en/docs/ProductDoc/MainBoard/fly-super/fly-super8-pro/pin/
- Mellow FLY-Super8 Pro Klipper configuration: https://mellow-3d.github.io/fly_super8_pro_h723_klipper_mcu_config.html
- Duet CAN-FD documentation: https://docs.duet3d.com/User_manual/Connecting_hardware/CAN-FD_bus

### Papers and research

- Robotic continuous-spatial-path FFF: https://doi.org/10.3390/app11114825 (2021, **OLD**)
- Five-axis support-free additive manufacturing: https://www.mdpi.com/2076-0825/14/2/99 (2025)
- Model decomposition for five-axis support-free fabrication: https://link.springer.com/article/10.1007/s12206-025-1044-1 (2025)
- RotBot slicing paper: https://www.mdpi.com/2076-3417/11/18/8760 (2021, **OLD**)
- Open5x related ACM paper: https://dl.acm.org/doi/10.1145/3491101.3519782
- Open5x related Additive Manufacturing paper: https://doi.org/10.1016/j.addma.2023.103546
