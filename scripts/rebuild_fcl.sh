#!/usr/bin/env bash
# Rebuild fcl 0.7.0 from source against the env's Eigen 5 headers and install
# the result over the conda-provided libfcl.
#
# Why: conda-forge's fcl 0.7.0 (built against Eigen 3) is ABI-incompatible
# with robostack's moveit-core 2.12.4 build _18 (requires eigen-abi >= 5.0.1.100,
# i.e. Eigen 5). On arm64/AAPCS64, Eigen 5 changed how 24-byte Eigen::Vector3d
# is returned (FP registers d0-d2) vs Eigen 3 (hidden x8 slot), so moveit's
# inlined computeLocalAABB() reads garbage -> SIGSEGV in PlanningScene::initialize().
# The `collision_detector: bullet` parameter cannot avoid this because FCL is
# hardcoded as the default allocator and is built before the param is read.
#
# This task rebuilds libfcl against the same Eigen 5.0.1 the rest of the moveit
# stack was compiled with, restoring ABI compatibility. It is idempotent: it
# skips when a marker file (stored in the env, wiped on env recreation) exists.
set -euo pipefail

FCL_VERSION=0.7.0
SRC="$PIXI_PROJECT_ROOT/.pixi/fcl-src-${FCL_VERSION}"
BUILD="$PIXI_PROJECT_ROOT/.pixi/fcl-build-${FCL_VERSION}"
PATCH="$PIXI_PROJECT_ROOT/patches/fcl-${FCL_VERSION}-eigen5-cxx17.patch"
MARKER="$CONDA_PREFIX/lib/.fcl-eigen5-rebuilt"

if [ -f "$MARKER" ]; then
  echo "[rebuild_fcl] already rebuilt against Eigen 5 (marker present), skipping"
  exit 0
fi

echo "[rebuild_fcl] building fcl ${FCL_VERSION} against Eigen 5 (C++17)..."

# Fetch source if not already cached (survives across runs, wiped with .pixi/)
if [ ! -d "$SRC" ]; then
  echo "[rebuild_fcl] cloning fcl ${FCL_VERSION}"
  git clone --depth 1 --branch "$FCL_VERSION" \
    https://github.com/flexible-collision-library/fcl.git "$SRC"
fi

# Apply the C++17 / cassert patch if not already applied (idempotent)
if ! git -C "$SRC" apply --reverse --check "$PATCH" >/dev/null 2>&1; then
  echo "[rebuild_fcl] applying patch $PATCH"
  git -C "$SRC" apply "$PATCH"
fi

echo "[rebuild_fcl] configuring"
cmake -S "$SRC" -B "$BUILD" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DCMAKE_C_COMPILER="$CONDA_PREFIX/bin/clang" \
  -DCMAKE_CXX_COMPILER="$CONDA_PREFIX/bin/clang++" \
  -DCMAKE_CXX_STANDARD=17 \
  -DEigen3_DIR="$CONDA_PREFIX/lib/cmake/eigen3" \
  -DCMAKE_SKIP_INSTALL_RPATH=ON \
  -DFCL_WITH_OCTOMAP=ON \
  -DBUILD_TESTING=OFF \
  -DFCL_STATIC_LIBRARY=OFF

echo "[rebuild_fcl] building"
cmake --build "$BUILD"

echo "[rebuild_fcl] installing over $CONDA_PREFIX/lib/libfcl*"
cmake --install "$BUILD"

touch "$MARKER"
echo "[rebuild_fcl] done: libfcl.0.7.0 rebuilt against Eigen 5"
