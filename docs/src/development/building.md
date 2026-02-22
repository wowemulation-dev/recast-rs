# Building

## Prerequisites

- Rust 1.92+ (2024 edition)
- `wasm32-unknown-unknown` target (for WASM builds)

Optional tools (provisioned by `.mise.toml`):

- `cargo-nextest`: Parallel test runner (used by CI)
- `cargo-llvm-cov`: Code coverage
- `cargo-deny`: Dependency auditing
- `mdbook`: Documentation site

## Build Commands

The workspace defines cargo aliases in `.cargo/config.toml`:

```bash
# Build
cargo build               # Debug build
cargo build-release       # Release build

# Check
cargo check-all           # Check all targets

# Test
cargo test-all            # Run all tests (nextest)
cargo test-lib            # Run library tests only (faster)
cargo test test_name      # Run a specific test

# Quality
cargo fmt --all           # Format code
cargo fmt-check           # Check formatting
cargo lint                # Clippy lints
cargo docs                # Generate documentation
```

## QA Pipeline

After completing any coding task, run the full QA suite:

```bash
cargo fmt --all && cargo lint && cargo test-all
```

CI runs these additional checks:

| Check | Command |
|-------|---------|
| Formatting | `cargo fmt --all --check` |
| Linting | `cargo clippy --workspace --all-targets` |
| Tests | `cargo nextest run --workspace --all-features` |
| Dependency audit | `cargo deny check --all-features` |
| No-default-features | `cargo check --workspace --no-default-features` |
| WASM compilation | `cargo build -p <crate> --target wasm32-unknown-unknown --no-default-features` |
| Documentation | `cargo doc --workspace --no-deps` |
| Coverage | `cargo llvm-cov --workspace --lcov` |

CI sets `RUSTFLAGS="-D warnings"` to treat warnings as errors. The same
flag is enabled in `.cargo/config.toml` for local development.

## Cross-Platform Builds

CI verifies compilation on 10 targets:

| Platform | Targets |
|----------|---------|
| Linux | x86_64-gnu, x86_64-musl, aarch64-gnu, aarch64-musl, armv7-gnueabihf, armv7-musleabihf |
| macOS | aarch64-apple-darwin, x86_64-apple-darwin |
| Windows | x86_64-msvc, x86_64-gnu |
| WASM | wasm32-unknown-unknown (library crates only) |

## Documentation

Build the API documentation:

```bash
cargo docs
```

Build this mdbook site:

```bash
cd docs && mdbook build
```

Serve locally with auto-reload:

```bash
cd docs && mdbook serve
```
