fn main() {
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/sfm/mod.rs");
    println!("cargo:rerun-if-changed=src/sfm/stage_3_7_bundle_adjustment.rs");
}
