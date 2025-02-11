set -e
cd "$(git rev-parse --show-toplevel)"
cmake --fresh --preset=release
cmake --build "cmake-build-release" --clean-first
