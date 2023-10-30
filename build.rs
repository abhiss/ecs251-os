fn main() {
    // Tell rustc to pass linker scripts to LLD
    // println!("cargo:rustc-link-arg=-Tmemory.x");
    println!("cargo:rustc-link-arg=-Tlinker.ld");
    
    // Rerun this script only when necesary
    println!("cargo:rerun-if-changed=linker.ld");
    println!("cargo:rerun-if-changed=build.rs");
}