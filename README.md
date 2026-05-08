# MotorCom

## Getting started

MotorCom is a communication library for Damiao/Robstride motors.

## Feature Overview

* Relies on CAN bus for communication
* Support for MIT mode
* Pure Rust plus python bindings (using [pyo3](https://pyo3.rs/)).

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

#### Robstride

```rust

```

## License

This library is licensed under the [Apache License 2.0](./LICENSE).