cc3DFin
========

<img width="892" alt="3dfin_logo" src="https://user-images.githubusercontent.com/68945855/233049674-8d2c96a7-8abc-4a7c-8e83-4a329ba6dd0c.png">

This is a "native" port (C++/Qt) of [3DFin](https://github.com/3DFin/3DFin) Python plugin.

It contains several improvements over the Python version:

- Speed improvements: better handling of parallel computations and near zero overhead communication with CloudCompare.
- New and better DBH computation: DBH computation in the Python version has some flaws (it always computes a MAD over 2 values). It was fixed in 3DFin 0.6.0 but not 100% satisfactory. This version comes with an (hopefully) better fix that leads to more DBH computations / results.
- Slightly better Z0 / height normalization: We compute height normalization using a smoothed DTM and an approximation of barycentric coordinates (instead of IDW, which was in fact a 'non Inverse DW' algorithm in the Python implementation).
- Slightly better circle fitting: more robust thanks to direct initialization and LM refinements with robust Huber loss function.
- Tilt outlier computation should be more correct.
- Better logging: Logging/progress is now reported in the plugin Widget and it does not spam the CC console.

Notable regressions w.r.t [3DFin](https://github.com/3DFin/3DFin) Python version (TODOs)

- No CLI / Standalone. In the long run `lib3DFin` is meant to be pulled off this monolithic repo to live its own life outside `CloudCompare` ;).
- No configuration file management. A port of 3DFin ini config file in toml/yaml is planned.
- Minimal / Incomplete / non-existent code documentation. Global code architecture will evolve quickly.

# Structure, code organization and dependencies

Most computation logic lives inside the `lib3DFin` directory. Computation algorithm is described by `lib3DFin/src/interface.cpp` file.
Dependencies are all included as subrepository inside `lib3DFin/third_party`, most of them are header only:

- [Eigen](https://gitlab.com/libeigen/eigen), The base linear algebra library.
- [CSF-3DFin](https://github.com/3DFin/CSF-3DFin) our fork of [CSF](https://github.com/jianboqi/CSF) that include backport from CC's CSF improvements.
- [nanoflann](https://github.com/jlblancoc/nanoflann) for fast KNN queries.
- [taskflow](https://github.com/taskflow/taskflow) a cross platform and convenient parallel computation library.
- [dset](https://github.com/wjakob/dset), header only union-find algorithm.
- [OpenXLSX](https://codeberg.org/lars_uffmann/OpenXLSX), Optional, needed for xlsx report generations. Overkill for our needs. Planned to be replaced by a simpler template + zip alternative.

# Compilation

The plugin requires at least `CloudCompare` 2.14 and a `C++20` compatible compiler. 

Pull this repository in `CloudCompare/plugins/private`.
```
git clone --recurse-submodules https://github.com/3DFin/cc3DFin
```

`cmake` options

`3DFin_EXPORT_XLSX` Enable to generate an `XLSX` report.

`3DFin_BUILD_SHARED` Enable to build 3DFin as a shared library (not fully validated and recommended to keep disabled, especially in Plugin context).

Recommended `cmake` flags:

besides the obvious `PLUGIN_STANDARD_3DFIN=ON` to enable cc3DFin plugin, some care should be taken to properly compile `XLSX` support.

```
  3DFin_EXPORT_XLSX=1
  CMAKE_POLICY_VERSION_MINIMUM=3.5 # an OpenXLSX submodule require and old version of cmake
```

# Usage

For usage please refers to the [documentation](https://github.com/3DFin/3DFin/blob/main/src/three_d_fin/documentation/documentation.pdf) and the [Tutorial](https://github.com/3DFin/3DFin_Tutorial) that should help you to get started. The GUI screenshots may differ slightly from the current state of `cc3DFin` but it still cover 99% of its features.

# Citing 3DFin

If you use 3DFin in your research, please cite the following paper:

Laino, D., Cabo, C., Prendes, C., Janvier, R., Ordonez, C., Nikonovas, T., Doerr, S., & Santin, C. (2024). 3DFin: a software for automated 3D forest inventories from terrestrial point clouds. Forestry: An International Journal of Forest Research. https://doi.org/10.1093/forestry/cpae020

Thank you for citing 3DFin in your work! Your citations help to support the continued development and maintenance of this software.

# References

CloudCompare-PythonRuntime, by Thomas Montaigu: [CloudCompare-PythonRuntime](https://github.com/tmontaigu/CloudCompare-PythonRuntime)

# Acknowledgement

3DFin has been developed at the Centre of Wildfire Research of Swansea University (UK) in collaboration with the Research Institute of Biodiversity (CSIC, Spain) and the Department of Mining Exploitation of the University of Oviedo (Spain).

Funding provided by the UK NERC project (NE/T001194/1):

'_Advancing 3D Fuel Mapping for Wildfire Behaviour and Risk Mitigation Modelling_'

and by the Spanish Knowledge Generation project (PID2021-126790NB-I00):

‘_Advancing carbon emission estimations from wildfires applying artificial intelligence to 3D terrestrial point clouds_’.
