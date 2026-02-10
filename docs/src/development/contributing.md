# Contributing

## Code Style

- Rust 2024 edition idioms
- 4-space indentation (see `.editorconfig`)
- Run `cargo fmt` before committing
- Fix all clippy warnings

## Error Handling

- Avoid `unwrap()` and `expect()` in new library code
- Return `Result<T, E>` from fallible functions
- Use `thiserror` for custom error types
- Each crate defines its own error type (not a shared workspace `Error`)
- Use structured variants with typed fields, not `String` payloads
- Existing `unwrap()`/`expect()` calls are a port artifact to be reduced

```rust,ignore
use thiserror::Error;

#[derive(Error, Debug)]
pub enum BuildError {
    #[error("too many vertices: {count} (max: {max})")]
    TooManyVertices { count: usize, max: usize },

    #[error("span position out of bounds: ({x}, {y})")]
    SpanOutOfBounds { x: i32, y: i32 },
}
```

See the [Resolution Roadmap](roadmap.md) section 1.2 for the planned
per-crate error type definitions.

## Memory Management

- Prefer borrowing over cloning
- Avoid unnecessary allocations in hot paths
- Use `Vec::with_capacity()` when size is known
- Profile before optimizing

## Testing

- Unit tests go in `#[cfg(test)]` modules in the same file
- CI uses `cargo-nextest` for parallel execution
- CI collects code coverage with `cargo-llvm-cov`

Run tests:

```bash
cargo test-all            # All tests
cargo test-lib            # Library tests only
cargo test test_name      # Specific test
```

## Unsafe Code

Unsafe code exists in three locations:

- `detour/src/nav_mesh.rs`: Disjoint mutable references, pointer offset
- `detour/src/node_pool.rs`: Raw pointer priority queue, manual `Send`/`Sync`
- `detour-dynamic/src/dynamic_tile.rs`: `get_unchecked` for span parsing

New unsafe code should be avoided. If necessary, document the safety invariants
with `// SAFETY:` comments.

## Workspace Lints

Clippy lint groups (`pedantic`, `nursery`, `cargo`) are configured in the
workspace `Cargo.toml` but set to `allow` for the initial port. They should
be tightened incrementally.

Current `allow` settings cover:
- `unsafe_code`: Used in performance-critical paths
- `unwrap_used`, `expect_used`, `panic`: Port artifact
- Algorithm-specific lints: Cast precision, float comparison

## Commit Messages

Follow [Conventional Commits v1.0.0](https://www.conventionalcommits.org/en/v1.0.0/).

```text
feat(detour): add hierarchical pathfinding
fix(recast): correct contour winding order
refactor(detour-crowd): split crowd update into phases
docs: add mdbook documentation site
chore(deps): upgrade glam to 0.31
```

## What to Work On

The [Resolution Roadmap](roadmap.md) lists all known issues ordered by
priority. Phase 1 (library safety) must be completed before crates.io
publication. Each item includes specific files, counts, and code examples.

## Algorithm Translation from C++

When modifying ported algorithms:

1. Preserve the same algorithmic structure as the C++ original
2. Use iterator/index operations instead of pointer arithmetic
3. Convert output parameters to return values
4. Rust provides bounds checking automatically
5. Test against the C++ implementation for correctness

Reference repositories:

- [C++ RecastNavigation](https://github.com/recastnavigation/recastnavigation)
- [DotRecast (.NET port)](https://github.com/ikpil/DotRecast)
