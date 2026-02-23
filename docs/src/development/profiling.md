# Performance Profiling

This guide explains how to profile recast-rs using flamegraphs and run tests with nextest.

## Flamegraph Profiling

Flamegraphs visualize CPU time allocation across call stacks, making it easy to identify performance bottlenecks in navmesh generation and pathfinding operations.

### Installation

Install `cargo-flamegraph`:

```bash
cargo install cargo-flamegraph
```

**Note**: On Linux, you may need `perf`:
- Debian/Ubuntu: `sudo apt-get install linux-tools-common`
- Fedora: `sudo dnf install perf`
- Arch: `sudo pacman -S perf`

macOS users need [Instruments](https://help.apple.com/instruments/) or alternative profilers.

### Generating Flamegraphs

#### Profile a Binary

```bash
cargo flamegraph --bin landmark-cli -- build input.obj output.bin
```

This generates `flamegraph.svg` showing CPU time for the navmesh generation pipeline.

#### Profile Tests

```bash
# Profile all tests
cargo flamegraph-test

# Profile a specific test
cargo flamegraph-test pathfinding -- --test-threads=1
```

Use `--test-threads=1` to avoid parallel execution interfering with profiling.

#### Profile Benchmarks

```bash
cargo flamegraph-bench --bench pathfinding
```

Analyzes performance of benchmark functions defined in `benches/` directories.

### Flamegraph Options

```bash
# Custom output file
cargo flamegraph --bin landmark-cli -o my-flamegraph.svg -- build input.obj output.bin

# Different frequency (lower = more samples)
cargo flamegraph --bin landmark-cli --freq 99 -- build input.obj output.bin

# With feature flags
cargo flamegraph --bin landmark-cli --features "tokio" -- build input.obj output.bin
```

### Interpreting Flamegraphs

The flamegraph shows:
- **X-axis**: Population (width = relative CPU time)
- **Y-axis**: Stack depth (each level is a function call)
- **Colors**: Random, distinguish adjacent frames

Common patterns to look for:
- Wide bars at the top = hot functions (optimization targets)
- Many narrow bars = function overhead
- Deep stacks = call chain complexity

### Example Use Cases

#### Identify Slow Pipeline Stage

```bash
cargo flamegraph --bin landmark-cli -- build dungeon.obj output.bin
```

Check which pipeline stage (heightfield, contours, polymesh) dominates.

#### Profile Pathfinding

```bash
cargo flamegraph-bench --bench pathfinding
```

Identify bottlenecks in A* search, neighbor queries, or funnel algorithm.

## Nextest Testing

[cargo-nextest](https://nexte.st/) is a faster test runner with better UX than `cargo test`.

### Installation

Nextest is already installed in CI. For local development:

```bash
cargo install cargo-nextest
```

### Running Tests

```bash
# Run all tests (default profile)
cargo nextest run --workspace

# Run tests in release mode (faster execution)
cargo nextest run --workspace --release

# Run specific crate tests
cargo nextest run -p waymark

# Run matching tests
cargo nextest run --test-threads=4 detour::pathfinding

# List all tests without running
cargo nextest list --workspace
```

### Profiles

Nextest uses configuration profiles defined in `.config/nextest.toml`:

#### Default Profile

Standard development mode with permissive timeouts:

```bash
cargo nextest run --profile default --workspace
```

#### CI Profile

Stricter timeouts and fail-fast for CI:

```bash
cargo nextest run --profile ci --workspace
```

#### Local Profile

More generous timeouts for local development:

```bash
cargo nextest run --profile local --workspace
```

#### Release Profile

For release-mode testing:

```bash
cargo nextest run --profile release --workspace --release
```

### Cargo Aliases

The project provides convenient aliases:

```bash
# Run all tests with nextest
cargo nextest-all

# Run tests in release mode
cargo nextest-release

# Run library tests only
cargo nextest-lib
```

### Nextest Features

#### Test Filtering

```bash
# Run tests matching a pattern
cargo nextest run pathfinding

# Run tests in a specific crate
cargo nextest run -p waymark

# Exclude tests
cargo nextest run --ignore 'integration'

# Filter by test type
cargo nextest run --lib        # Library tests only
cargo nextest run --bins       # Binary tests only
```

#### Test Execution Control

```bash
# Stop after first failure
cargo nextest run --fail-fast --workspace

# Continue on failure (default)
cargo nextest run --no-fail-fast --workspace

# Retry failed tests
cargo nextest run --retries 2 --workspace
```

#### Verbose Output

```bash
# Show test output
cargo nextest run --workspace --success-output=immediate

# Show only failing test output
cargo nextest run --workspace --failure-output=immediate-final

# Show all test output at end
cargo nextest run --workspace --success-output=final
```

### Integration with Flamegraph

Combine flamegraph and nextest to profile specific tests:

```bash
cargo flamegraph-test --test-threads=1 -- exact::test_name
```

The `--test-threads=1` flag is important for accurate profiling with flamegraph.

## Performance Tuning Workflow

1. **Baseline**: Run benchmarks with `cargo bench`
2. **Profile**: Generate flamegraph with `cargo flamegraph`
3. **Analyze**: Identify hot functions in the flamegraph
4. **Optimize**: Modify code to target identified bottlenecks
5. **Verify**: Re-run benchmarks to confirm improvement
6. **Regression test**: Run `cargo nextest-all` to ensure no functional breakage

## Continuous Integration

CI uses nextest with the `ci` profile for faster, more reliable test execution. The configuration is in `.config/nextest.toml`.

CI does not generate flamegraphs automatically due to time and resource constraints.
