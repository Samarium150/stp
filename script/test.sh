set -e
cd "$(git rev-parse --show-toplevel)"

if [ -d "cmake-build-release/test" ]; then
    cd "cmake-build-release/test"
else
    ./script/build.sh
    cd "cmake-build-release/test"
fi

testBinaries=(
    # "test_pdb_standard_puzzle"
    # "test_manhattan_standard_puzzle"
    # "test_manhattan_modified_puzzle"
    "test_pdb_modified_puzzle"
)

for binary in "${testBinaries[@]}"; do
    if [ -f "./$binary" ]; then
        ./"$binary" 2>&1 | tee "$binary.txt"
    fi
done
