fn main() {
    println!("cargo:rustc-link-lib=cudart");
    if let Ok(path) = std::env::var("CUDA_PATH") {
        println!("cargo:rustc-link-search={}/lib64", path);
    }
    println!("cargo:rustc-link-search=/usr/local/cuda/lib64");
    println!("cargo:rustc-link-search=/usr/lib/x86_64-linux-gnu");
}
