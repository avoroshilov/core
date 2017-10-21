# Core files
This project is a unified set of core files used by some of the projects that I am working on.

Files of this project were created a long time ago, many of them are largely unmodified since then, but were sanitized at least to some degree, which explains appearance of C++1x here and there.

## Contents
Core contains:
1. Math - 2,3 and 4 dimensional vector classes, 3×4 and 4×4 matrices
2. Formats - simple TGA and WAV file writers
3. Texture - fast tileable power-of-2 texture synthesis:
  1. generators - fractal plasm, voronoi cells, bricks, radial gradients, etc.
  2. processors - map distortions, rotozoom, etc.
  3. color manipulations - brightness/contrast, convolution filters (various blurs, sobel, emboss, and others), HSV adjustments, etc.
  4. layer blending operations
4. SFX - sound synthesis and processing classes:
  1. generators - oscillators and noises
  2. 2nd order IIR filters
  3. delay lines
  4. derived effects such as Chorus, Flanger, Phaser
  5. 4-channel FDN reverb
5. Windows OS specific helpers:
  1. timer
  2. window manipulations
  3. working with WaveOut (including spatial positioning)

## Projects
Projects that **do not** use this unified Core:
  1. Verilog MIPS pipelined processor
  2. Coupled simulation of FEM and rigid body dynamics with joints
  3. Python/TF optimization-based neural style transfer
  4. Python/TF Squeezenet implementation
  5. TeX paper mark up
  6. C++ neural networks framework (not yet present on github)
  7. Python/TF reinforced learning experiments (not yet present on github)

Projects that **use** the unified Core:
  1. Standalone texture synthesis sample
  2. Standalone sound synthesis sample
  3. Simple software rendering
  4. Vulkan base code
  5. GLSL path tracer (derivative of Vulkan base)
  6. OpenGL graphics engine with real-time physics simulation (not yet present on github - parts of the Core that are used by this project are also not yet present - see **Future work** section)

## Future work
Sanitize and share the biggest missing parts of the Core subsystems:
  1. Broad- and narrow-phase collision detection and raycasting (dpends on the helper radix sorting)
  2. Dynamics simulations - rigid body (joints jacobian calculations and impulse-based PGS solver), fluids (lagrangian - SPH), deformables (position-based mass-spring - additionally incorporate impulse-based FEM from the older collaboration research project)
  3. Container-based retained GUI system
  4. Render manager - unify OpenGL and Vulkan base code rendering backend interface
  5. Helper algorithms, such as various sorting algorithms
