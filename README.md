# Chasm City Cable Car

## Overview
A 2D take on the cable cars of Alastair Reynolds' *Chasm City*. Vines hang from a canopy of corrupted buildings, some free, some draped between two points, some knotted into other vines. A pod with four telescopic arms climbs through them: the arms reach for the vines with their hooks, carry the pod, and let go again to move along the canopy. A controller keeps the pod stable while it moves to the position given by the mouse or the arrow keys.

## Installation
Dependencies: a C++17 compiler and [SDL3](https://github.com/libsdl-org/SDL) (for the web build [emsdk](https://emscripten.org), which ships an SDL3 port).

Build:
```
chmod +x build.sh
./build.sh
```
On Windows via WSL (Ubuntu 24.04 has no SDL3 package, see the script header to build it locally):
```
./build_windows.sh           # native + wasm
./build_windows.sh native    # native binary only
./build_windows.sh wasm      # cable_car.js/.wasm for the website
```
Run with:
```
./bin/cable_car
```

## Controls
Mouse - desired pod position
Arrow keys - move the desired position
C - toggle controller (pod hangs passively from its grips)
F - toggle path finding (off: straight towards the target)
W - toggle wind
T - toggle the trail drawn by the pod
D - debug view (reach, preferred grips, arm forces, reference, supported area)
R - respawn the pod
G - grow new vines
P - screenshot (native)
Space - pause
Q / Esc - quit

## Skyline growth demo
A standalone demo (`bin/growth_demo`, `growth_demo.html`) for the background: futuristic skyscrapers whose tops morph into tree-like crowns, drawn in the same translucent style as the canopy. It will later be merged into the game, with the cables hanging from the crowns.

Controls: Space - pause, R - restart, G - new skyline, Up/Down - growth speed, D - show nodes (brightness = growth mask), P - screenshot, Q / Esc - quit

Every building is a closed outline of nodes (counter clockwise). Differential growth moves them each step:
- springs keep the edges at their rest length, smoothing pulls a node to the midpoint of its neighbours
- repulsion (spatial hash) keeps all nodes of a layer apart, so outlines never cross
- new nodes are inserted mostly at growing tips: convex parts of the outline facing up in the middle and up-and-out at the sides, so bumps become branches instead of coral-like folds
- a growth mask is 0 on the lower 60 % of the tower (unchanged skyscraper) and rises to 1 towards the top
- growth slows down and stops at 2.5 times the initial node count, the simulation stops once everything is at rest

Nodes are never removed, only inserted, so a node index is a stable attachment point that moves with the growth.

## Details

### Vines
The vines are chains of particles simulated with position based dynamics (30 substeps per frame, distance constraints between neighbours, a soft bending constraint over two segments). Anchors on the canopy are particles with infinite mass. The canopy itself is a randomly grown branching structure, its nodes are the anchor points.

### Pod and arms
The pod is a rigid body (position, orientation) with 40 kg. Each arm consists of two links which telescope together; inverse kinematics (law of cosines, like the double pendulum robot arm) places the elbow away from the pod, the telescope keeps the elbow at a comfortable bend. Every arm can apply a limited force between its shoulder and the hook; the reaction force acts on the vine, so the pod's weight pulls the vines down. When an arm is stretched to its limit a stiff mechanical stop engages.

### Controller
A reference point moves towards the target with limited speed and acceleration. A PD law on the reference plus gravity compensation gives the desired force, a PD law on the orientation the desired torque. This wrench is distributed onto the grasping arms by a weighted least norm solution (forces along an arm are cheaper than across it) and limited to the motor strength. When the arms are too weak for everything, leveling keeps priority over carrying the weight, which keeps priority over moving; the torque of the mechanical stops is compensated.

### Path finding
The world is covered by a 1 m grid. For every cell the planner counts the distinct vines a pod there could reach (from the current, moving cable positions). Cells with at least three vines are supported. Dijkstra from the pod finds everything reachable over supported cells, with steps through weakly supported cells costing more, so wide, dense corridors are preferred. The goal is the reachable cell closest to the target (the target itself if possible); the path is shortened by string pulling and, if the target lies outside the supported area, the last bit is taken directly. The controller follows a point 3 m ahead on the path; the path is renewed every 0.5 s or when the target moves.

### Gait
Every arm has a preferred grip: out in its natural direction and shifted towards the target. Grip candidates on all vines within reach are scored by their distance to it, with penalties for crowding other hooks, reaching across the pod and floppy vine tips.
- free arms reach for the best candidate
- grasping arms slide along their vine towards a better spot (climbing)
- if at least three arms hold on, the one whose grip can be improved the most lets go and reaches for the better grip
