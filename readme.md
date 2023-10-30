## Installing:
```
rustup toolchain install nightly
rustup toolchain link custom-rv32e ./rust/build/host/stage1
```

## Building: 
```
cargo +custom-rv32e -Zbuild-std=core build
```

## Commands that might be useful:
```
llvm-objdump-17  ./target/riscv32em-unknown-none-elf/debug/os -h
llvm-objdump-17  ./target/riscv32em-unknown-none-elf/debug/os -s --section=.text
rustup component add rust-src --toolchain nightly-x86_64-unknown-linux-gnu
```
