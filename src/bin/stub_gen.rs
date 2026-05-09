//! Generate Python type stubs (`.pyi`) for the `motorcom` extension module.
//!
//! Walks every `#[gen_stub_*]`-annotated item in `src/python.rs` and writes
//! out `motorcom.pyi` next to the wheel so type checkers (mypy, pyright) and
//! IDEs get accurate signatures.
//!
//! Run with:
//! ```text
//! cargo run --bin stub_gen --features python
//! ```

fn main() -> pyo3_stub_gen::Result<()> {
    let stub = motorcom::python::stub_info()?;
    stub.generate()?;
    Ok(())
}
