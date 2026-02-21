#!/usr/bin/env bash
#
# Compare Recast C++ and Rust CLI build performance.
#
# Usage:
#   ./scripts/bench-compare.sh [OPTIONS]
#
# Options:
#   -m, --mesh FILE       Input mesh file (default: test-data/meshes/nav_test.obj)
#   -n, --iterations N    Iterations per bench run (default: 10)
#   --cpp-bin PATH        Path to C++ RecastCLI binary
#   --rust-bin PATH       Path to Rust recast-cli binary
#   --hyperfine-runs N    Number of hyperfine runs (default: 5)
#   -h, --help            Show this help

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Defaults
MESH="${PROJECT_ROOT}/test-data/meshes/nav_test.obj"
ITERATIONS=10
CPP_BIN=""
RUST_BIN=""
HYPERFINE_RUNS=5

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -m|--mesh)       MESH="$2"; shift 2 ;;
        -n|--iterations) ITERATIONS="$2"; shift 2 ;;
        --cpp-bin)       CPP_BIN="$2"; shift 2 ;;
        --rust-bin)      RUST_BIN="$2"; shift 2 ;;
        --hyperfine-runs) HYPERFINE_RUNS="$2"; shift 2 ;;
        -h|--help)
            head -14 "$0" | tail -13
            exit 0
            ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

# Auto-detect binaries if not specified
if [[ -z "$CPP_BIN" ]]; then
    # Try common locations
    for candidate in \
        "${PROJECT_ROOT}/../recastnavigation/build/linux-release/bin/RecastCLI" \
        "$(command -v RecastCLI 2>/dev/null || true)"; do
        if [[ -n "$candidate" && -x "$candidate" ]]; then
            CPP_BIN="$candidate"
            break
        fi
    done
fi

if [[ -z "$RUST_BIN" ]]; then
    # Build release if needed
    cargo build --release -p recast-cli --manifest-path="${PROJECT_ROOT}/Cargo.toml" 2>/dev/null
    RUST_BIN="${PROJECT_ROOT}/target/release/recast-cli"
fi

# Validate
if [[ ! -f "$MESH" ]]; then
    echo "ERROR: Mesh file not found: $MESH" >&2
    exit 1
fi

if [[ -z "$CPP_BIN" || ! -x "$CPP_BIN" ]]; then
    echo "ERROR: C++ RecastCLI binary not found. Use --cpp-bin to specify." >&2
    exit 1
fi

if [[ ! -x "$RUST_BIN" ]]; then
    echo "ERROR: Rust recast-cli binary not found at: $RUST_BIN" >&2
    exit 1
fi

MESH_NAME="$(basename "$MESH")"

echo "============================================"
echo " Recast C++ vs Rust Benchmark Comparison"
echo "============================================"
echo ""
echo "Mesh:       $MESH_NAME"
echo "Iterations: $ITERATIONS (per bench run)"
echo "C++ binary: $CPP_BIN"
echo "Rust binary: $RUST_BIN"
echo ""

# ---------------------------------------------------------------------------
# 1. Internal bench command (per-stage breakdown)
# ---------------------------------------------------------------------------

echo "--------------------------------------------"
echo " C++ bench (${ITERATIONS} iterations)"
echo "--------------------------------------------"
"$CPP_BIN" bench --input "$MESH" --iterations "$ITERATIONS" 2>&1 | grep -E '^\s+(Min|Max|Mean|Median|StdDev|Rasterize|Filter|Build|Erode|===|-)' || true
echo ""

echo "--------------------------------------------"
echo " Rust bench (${ITERATIONS} iterations)"
echo "--------------------------------------------"
"$RUST_BIN" bench --input "$MESH" --iterations "$ITERATIONS" 2>&1 | grep -E '^\s+(Min|Max|Mean|Median|StdDev|Rasterize|Filter|Build|Erode|===|-)' || true
echo ""

# ---------------------------------------------------------------------------
# 2. hyperfine end-to-end comparison (if available)
# ---------------------------------------------------------------------------

if command -v hyperfine &>/dev/null; then
    echo "--------------------------------------------"
    echo " hyperfine end-to-end comparison"
    echo "--------------------------------------------"
    echo ""

    hyperfine \
        --runs "$HYPERFINE_RUNS" \
        --warmup 2 \
        --export-markdown /dev/stdout \
        -n "C++ RecastCLI" "'$CPP_BIN' bench --input '$MESH' --iterations 1 2>/dev/null" \
        -n "Rust recast-cli" "'$RUST_BIN' bench --input '$MESH' --iterations 1 2>/dev/null"
else
    echo "(hyperfine not found, skipping end-to-end comparison)"
    echo "Install with: cargo install hyperfine"
fi
