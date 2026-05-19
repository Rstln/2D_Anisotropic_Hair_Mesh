# Hair2D

Hair2D is a small C++/OpenGL research prototype for simulating a 2D hair mesh with adaptive tearing. It combines an XPBD-style solver, circular colliders, dynamic crack topology, and a color visualization of strand length error.

## Features

- 2D hair bundle represented by layered left/right boundary vertices.
- Adaptive crack creation, upward propagation, and merge-back logic.
- XPBD-inspired length and bending constraints.
- Circle collider interaction with boundary and crack edges.
- Thick-line OpenGL rendering for the mesh frame, strands, and colliders.
- Heat-map coloring for strand stretch/compression.
- Capture mode for generating a README/demo screenshot.

## Project Layout

- `src/main.cpp` - demo scene, fixed-step loop, collider animation, screenshot capture.
- `src/XPBDSimulator.cpp` - prediction, adaptive tearing, constraints, and collision solving.
- `src/Bundle.cpp` - hair mesh topology and render buffer generation.
- `src/Crack.cpp` - crack node creation, propagation, merge, and synchronization.
- `src/Collider.cpp` - circular collider geometry and rendering.
- `shaders/` - OpenGL shader programs.
- `external/glad/` - vendored GLAD loader.

## Build

This project currently targets macOS with GLFW and GLM installed through Homebrew.

```bash
cmake -S . -B cmake-build-debug
cmake --build cmake-build-debug
```

If your GLFW or GLM installation lives somewhere else, update the include/link paths in `CMakeLists.txt` or replace them with your local CMake package setup.

## Run

```bash
./cmake-build-debug/hair2d
```

Press `Esc` to close the demo window.

## Capture A Preview

```bash
./cmake-build-debug/hair2d --capture
```

By default this runs the simulation briefly, writes `output/preview.png`, and exits.

After generating a preview image, you can add it to the README near the top with:

```markdown
![Hair2D preview](output/preview.png)
```

You can also choose a path:

```bash
./cmake-build-debug/hair2d --capture-path output/custom_preview.png
```

## Notes

The code is intentionally compact and experimental. The most important model concepts are:

- `u_rest` stores the original horizontal parameter of a vertex.
- `u` stores the current topological/interpolated position along a layer.
- `vtx_matrix` is refreshed from the current crack map and used by both simulation queries and rendering.
- Crack tips are represented as shared left/right endpoints until the crack propagates upward and splits the old tip.

Future improvements could include JSON scene configs, automated numerical tests for constraint residuals, and a portable CMake dependency setup.
