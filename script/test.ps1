$ErrorActionPreference = "Stop"

$repoRoot = (git rev-parse --show-toplevel).Trim()
Set-Location $repoRoot

if (Test-Path "cmake-build-release\test" -PathType Container) {
    Set-Location "cmake-build-release\test"
} else {
    & ".\script\build.ps1"
    Set-Location "cmake-build-release\test"
}

$testBinaries = @(
    "test_manhattan_standard_puzzle.exe",
    "test_manhattan_modified_puzzle.exe",
    "test_pdb_standard_puzzle.exe",
    "test_pdb_modified_puzzle.exe"
)

foreach ($binary in $testBinaries) {
    if (Test-Path ".\$binary" -PathType Leaf) {
        & ".\$binary" 2>&1 | Tee-Object -FilePath "$binary.txt"
    }
}
