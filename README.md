# MotorCom

## Getting started

MotorCom is a communication library for Damiao/Robstride motors.

## Feature Overview

* Relies on CAN bus for communication
* Support for MIT mode
* Pure Rust plus python bindings (using [pyo3](https://pyo3.rs/)).

---

## Dependencies

```bash
sudo apt update
sudo apt install -y libudev-dev
```

---

## APIs

It exposes two layers:
* Low-level protocol handlers: handle the CAN bus communication and packet parsing. Useful for fine-grained control of a shared bus.
  * `ProtocolHandler` — protocol (constructed with `::new()`).
* `Controller`: high-level API per motor.

See the examples below for usage.

### Examples

#### Damiao

```rust
use motorcom::motor::damiao::dm4340p::DMController;
use std::time::Duration;

fn main() {
    let can_bus = can::new("can0", 1_000_000)
        .timeout(Duration::from_millis(1000))
        .open()
        .unwrap();

    let mut c = DMController::new().with_bus(can_bus);
    let pos = c.read_position(&vec![1, 2]).unwrap();
    println!("Motor position: {:?}", pos);
    c.sync_write_goal_position(&vec![1, 2], &vec![1000, 2000]).unwrap();
}
```

##### Scanning a CAN bus for Damiao motors

`examples/damiao_scan.rs` walks a range of candidate motor ids and prints the
`motor_id` / `master_id` pair for every motor that responds, along with its
decoded MIT status (position, velocity, torque, temps). The status decoding
uses the DM-J4340P-2EC (`DM4340P`) limits — swap the constant in the example
for a different variant if needed.

Bring the CAN interface up first (Damiao runs at 1 Mbit classic CAN):

```bash
sudo ip link set can0 up type can bitrate 1000000
```

Then run the scanner. Args (all optional): `<interface> <start_id> <end_id>`,
defaulting to `can0 1 64`:

```bash
cargo run --release --example damiao_scan                 # scan 0x01..=0x40 on can0
cargo run --release --example damiao_scan -- can0 1 127   # full standard-id range
cargo run --release --example damiao_scan -- can1 1 16    # different bus, narrower range
```

Sample output:

```text
  motor_id  master_id    pos[rad]   vel[rad/s]  tau[N·m]   Tmos   Trot
  ------------------------------------------------------------------------
    0x01       0x11       +0.000       +0.000      +0.00     32     34
    0x02       0x12       +1.234       +0.000      +0.00     31     33
done — 2 motor(s) responded
```

Under the hood the example broadcasts a feedback-request frame (`D[2]=0xCC`)
on the Damiao register id `0x7FF` for each candidate id; the motor replies on
its configured master id with an 8-byte MIT status frame, which the scanner
captures to recover the `master_id` mapping.

#### Robstride

```rust

```

---

## Development

### Install Rust & Cargo

Install build deps first (needed by socketcan and serialport):

```bash
sudo apt update
sudo apt install -y build-essential pkg-config libudev-dev
```

Don't use apt install rustc — Debian's package is usually too old for modern crates.

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"

#Pick option 1 (default) at the prompt. Then verify:

rustc --version
cargo --version
```

```bash
# 让 rustup 本身和所有 toolchain 下载都走国内
export RUSTUP_DIST_SERVER="https://rsproxy.cn"
export RUSTUP_UPDATE_ROOT="https://rsproxy.cn/rustup"                                                                                                                    

# 用 rsproxy 镜像的安装脚本
curl --proto '=https' --tlsv1.2 -sSf https://rsproxy.cn/rustup-init.sh | sh

# 装完加进当前 shell
source "$HOME/.cargo/env"
```

### Rust & Cargo Uninstallation

If you installed rustc or cargo via `apt-get` before, remove it first:

```bash
sudo apt remove --purge rustc cargo
sudo apt autoremove
```

### Build

Build this Rust:

```bash
cargo build --release

# If a Raspberry Pi is short on memory (Pi Zero / 3B with 1 GB RAM), building may run out of memory (OOM). You can limit the parallelism:     
cargo build --release -j 1

# Build with examples
cargo build --release --examples
```

---

## License

This library is licensed under the [Apache License 2.0](./LICENSE).