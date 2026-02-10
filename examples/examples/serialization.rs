//! Serialization example.
//!
//! Demonstrates saving and loading a NavMesh in JSON and binary (postcard)
//! formats. Shows in-memory round-trip and compares sizes.
//! Requires the `serialization` feature on the `detour` crate.
//!
//! Run with:
//! ```sh
//! cargo run -p recast-rs-examples --example serialization
//! ```

use recast_rs_examples::common;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (nav_mesh, _poly_mesh, _detail_mesh) =
        common::build_navmesh_from_obj("examples/data/nav_test.obj")?;

    println!();

    // Serialize to JSON bytes (in-memory)
    let json_bytes = nav_mesh.to_json_bytes()?;
    println!(
        "JSON size: {} bytes ({:.1} KB)",
        json_bytes.len(),
        json_bytes.len() as f64 / 1024.0
    );

    // Serialize to binary bytes (postcard format, in-memory)
    let binary_bytes = nav_mesh.to_binary_bytes()?;
    println!(
        "Binary size: {} bytes ({:.1} KB)",
        binary_bytes.len(),
        binary_bytes.len() as f64 / 1024.0
    );

    let ratio = json_bytes.len() as f64 / binary_bytes.len() as f64;
    println!("JSON/binary ratio: {:.1}x", ratio);

    println!();

    // Round-trip: JSON
    let restored_json = detour::NavMesh::from_json_bytes(&json_bytes)?;
    println!(
        "JSON round-trip: restored NavMesh from {} bytes",
        json_bytes.len()
    );

    // Round-trip: binary
    let restored_binary = detour::NavMesh::from_binary_bytes(&binary_bytes)?;
    println!(
        "Binary round-trip: restored NavMesh from {} bytes",
        binary_bytes.len()
    );

    // Verify round-trip by re-serializing and comparing sizes
    let json_bytes_2 = restored_json.to_json_bytes()?;
    let binary_bytes_2 = restored_binary.to_binary_bytes()?;

    println!();
    println!("Verification:");
    println!(
        "  JSON:   original={} bytes, re-serialized={} bytes, match={}",
        json_bytes.len(),
        json_bytes_2.len(),
        json_bytes.len() == json_bytes_2.len()
    );
    println!(
        "  Binary: original={} bytes, re-serialized={} bytes, match={}",
        binary_bytes.len(),
        binary_bytes_2.len(),
        binary_bytes == binary_bytes_2
    );

    // File-based round-trip using a temporary directory
    let temp_dir = std::env::temp_dir().join("recast-rs-example");
    std::fs::create_dir_all(&temp_dir)?;

    let json_path = temp_dir.join("navmesh.json");
    let bin_path = temp_dir.join("navmesh.bin");

    nav_mesh.save_to_json(&json_path)?;
    nav_mesh.save_to_binary(&bin_path)?;

    let json_file_size = std::fs::metadata(&json_path)?.len();
    let bin_file_size = std::fs::metadata(&bin_path)?.len();

    println!();
    println!("File-based serialization:");
    println!(
        "  JSON file: {} ({} bytes)",
        json_path.display(),
        json_file_size
    );
    println!(
        "  Binary file: {} ({} bytes)",
        bin_path.display(),
        bin_file_size
    );

    let _loaded_json = detour::NavMesh::load_from_json(&json_path)?;
    let _loaded_binary = detour::NavMesh::load_from_binary(&bin_path)?;
    println!("  Both files loaded back successfully");

    // Clean up temp files
    let _ = std::fs::remove_file(&json_path);
    let _ = std::fs::remove_file(&bin_path);
    let _ = std::fs::remove_dir(&temp_dir);

    Ok(())
}
