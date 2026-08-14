# app/models — Algorithm / Inference Models

This directory hosts **algorithm model packages** vendored into the DynamicAlgoCam
project. Each subdirectory is one fully self-contained model package (source +
license + requirements) so the project can run inference offline without external
pip installs, and so future algorithms can be added with a predictable layout.

## Layout convention

```
app/models/
└── <model_name>/                # e.g. yolov8
    ├── README.md                # (from upstream) — original model docs
    ├── LICENSE                  # upstream license — MUST be preserved
    ├── requirements.txt         # Python deps for this model only
    ├── setup.py / setup.cfg     # if upstream ships them
    └── <package>/               # the actual Python source tree
```

Each model subdirectory is independent — bringing in a new model never touches
the C++ build (the C++ `dynamic_algo_cam` binary under `app/dynamic_algo_cam/`
stays unaware of `app/models/` unless a future task explicitly bridges the two).

## Why this is separate from `vendors/`

| Directory | Content | Built by |
|-----------|---------|----------|
| `vendors/` | Third-party **C++ SDKs** consumed by the C++ build | `add_subdirectory()` in CMake |
| `app/models/` | Third-party **Python algorithm / inference packages** | standalone Python (`pip install -e ./app/models/<name>` or `python -m <package>`) |

Keeping them separate prevents the C++ build (`app/CMakeLists.txt`) from ever
trying to `add_subdirectory()` a Python package, and keeps the `vendors/`
semantic ("C++ SDKs that the binary links against") intact.

## Currently vendored

| Subdir | Upstream | Version | License |
|--------|----------|---------|---------|
| `yolov8/` | [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) | 8.0.29 (see `yolov8/ultralytics/__init__.py`) | **GPL-3.0** (see `yolov8/LICENSE`) |

⚠️ **License disclosure — required reading:** YOLOv8 is distributed under the
**GNU General Public License v3.0** which is *not* compatible with the host
project's MIT license. Distributing a combined work that incorporates YOLOv8
source code triggers GPL-3.0 obligations (source availability, notice
propagation, etc.). See the project root `README.md` → **License** section for
the full disclosure text, and `app/models/yolov8/LICENSE` for the upstream
license text.

## Adding a new model

1. Create `app/models/<new_name>/` and copy the upstream package into it.
2. Preserve upstream `LICENSE` and `requirements.txt`.
3. Add a row to the **Currently vendored** table above (upstream URL, version,
   license).
4. Update the root `README.md` License section if the new license differs
   from MIT.
5. (Optional) Add `docs/dynamic_algo_cam/<new_name>_overview.md` describing how
   the model is invoked and how it integrates with the capture pipeline.
