# Traffic-Flow-Optimizer-

A small C++ command-line program that simulates a traffic network with adjustable traffic lights, congestion factors, and routing for emergency vehicles. The program models a fixed 10-node road network, lets you input traffic and green-times for nodes, simulates traffic flow cycles, and computes emergency routes (Dijkstra) and a minimum spanning traversal (Kruskal-based MST).

## Features

- Fixed sample road network (10 nodes) with weighted edges.
- Dynamic traffic lights that adjust green times proportionally to directional traffic.
- Congestion factor support on edges (rain/accident or manual factor).
- Dijkstra shortest-path for emergency vehicle routing (time estimate uses 45 km/h speed assumption).
- MST-based contiguous route extraction to get a minimal-route traversal.
- Interactive CLI menu to simulate traffic cycles, add traffic, view network, and perform pathfinding.

## Files

- `ATS.cpp` — main program source. Contains Graph, TrafficLight, TrafficFlow, UI and main.
- `README.md` — this file.

## Requirements

- C++ compiler supporting C++17 (gcc/clang or MSVC). The code uses standard C++ library only.

## Build

Open PowerShell in the folder containing `ATS.cpp` (for example, your Desktop) and build with one of the following commands.

Using g++ (MinGW/MSYS or WSL):

```powershell
g++ -std=c++17 -O2 "Path\ATS.cpp" -o "Path\ATS.exe"
```

Using clang++:

```powershell
clang++ -std=c++17 -O2 "Path\ATS.cpp" -o "Path\ATS.exe"
```

Using MSVC (Developer Command Prompt / vcvars initialized). In PowerShell adapt paths as needed:

```powershell
cl /std:c++17 /O2 "Path\ATS.cpp" /Fe:"Path\ATS.exe"
```

## Run

From PowerShell:

```powershell
# if in Desktop folder
.\
