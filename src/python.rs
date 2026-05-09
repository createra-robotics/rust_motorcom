//! Python bindings via pyo3.
//!
//! Built only with `--features python`. Exposes a `motorcom` Python module
//! whose API mirrors the Rust crate as closely as practical:
//!
//! ```python
//! import motorcom as mc
//! bus = mc.SocketCanTransport("can0")
//! dm = mc.Damiao(motor_id=1, master_id=0x11, limits=mc.MitLimits.dm4310())
//! dm.enable(bus)
//! dm.mit_control(bus, mc.MitSetpoint(q=0.0, kp=50.0, kd=1.0))
//! dm.disable(bus)
//! ```
//!
//! `cargo run --bin stub_gen --features python` writes `.pyi` files for type
//! checkers / IDE autocompletion via pyo3-stub-gen.

use std::sync::Mutex;
use std::time::Duration;

use pyo3::exceptions::{PyIOError, PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pyfunction, gen_stub_pymethods};

use crate::damiao::mit::{MitLimits, MitSetpoint, MotorState};
use crate::damiao::{
    self, Damiao, DM10010, DM10010L, DM3507, DM4310, DM4310P, DM4340, DM4340P,
    DM6006, DM6248P, DM8006, DM8009, DMG6215, DMH3510, DMH6220, DMJH11,
};
use crate::transport::CanError;

#[cfg(feature = "socketcan-backend")]
use crate::damiao::ControlMode;
#[cfg(feature = "socketcan-backend")]
use crate::transport::{CanTransport, SocketCanTransport};

// ---------------------------------------------------------------------------
// Error mapping
// ---------------------------------------------------------------------------

fn can_err_to_py(e: CanError) -> PyErr {
    match e {
        CanError::Timeout => PyIOError::new_err("CAN timeout"),
        CanError::OversizedFrame(n) => PyValueError::new_err(format!("oversized frame: {n} bytes")),
        CanError::InvalidId => PyValueError::new_err("invalid CAN id"),
        CanError::Backend(io) => PyIOError::new_err(io.to_string()),
    }
}

fn dm_err_to_py(e: damiao::Error) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

// ---------------------------------------------------------------------------
// MitLimits
// ---------------------------------------------------------------------------

#[gen_stub_pyclass]
#[pyclass(name = "MitLimits", module = "motorcom")]
#[derive(Clone, Copy)]
pub struct PyMitLimits {
    pub(crate) inner: MitLimits,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyMitLimits {
    #[new]
    #[pyo3(signature = (p_max=12.5, v_max=30.0, kp_max=500.0, kd_max=5.0, t_max=10.0))]
    fn new(p_max: f32, v_max: f32, kp_max: f32, kd_max: f32, t_max: f32) -> Self {
        Self { inner: MitLimits { p_max, v_max, kp_max, kd_max, t_max } }
    }

    #[getter] fn p_max(&self)  -> f32 { self.inner.p_max  }
    #[getter] fn v_max(&self)  -> f32 { self.inner.v_max  }
    #[getter] fn kp_max(&self) -> f32 { self.inner.kp_max }
    #[getter] fn kd_max(&self) -> f32 { self.inner.kd_max }
    #[getter] fn t_max(&self)  -> f32 { self.inner.t_max  }

    fn __repr__(&self) -> String {
        let l = self.inner;
        format!(
            "MitLimits(p_max={}, v_max={}, kp_max={}, kd_max={}, t_max={})",
            l.p_max, l.v_max, l.kp_max, l.kd_max, l.t_max
        )
    }

    // Per-variant presets (match the Rust constants).
    #[staticmethod] fn dm3507()   -> Self { Self { inner: DM3507   } }
    #[staticmethod] fn dm4310()   -> Self { Self { inner: DM4310   } }
    #[staticmethod] fn dm4310p()  -> Self { Self { inner: DM4310P  } }
    #[staticmethod] fn dm4340()   -> Self { Self { inner: DM4340   } }
    #[staticmethod] fn dm4340p()  -> Self { Self { inner: DM4340P  } }
    #[staticmethod] fn dm6006()   -> Self { Self { inner: DM6006   } }
    #[staticmethod] fn dm6248p()  -> Self { Self { inner: DM6248P  } }
    #[staticmethod] fn dm8006()   -> Self { Self { inner: DM8006   } }
    #[staticmethod] fn dm8009()   -> Self { Self { inner: DM8009   } }
    #[staticmethod] fn dm10010()  -> Self { Self { inner: DM10010  } }
    #[staticmethod] fn dm10010l() -> Self { Self { inner: DM10010L } }
    #[staticmethod] fn dmg6215()  -> Self { Self { inner: DMG6215  } }
    #[staticmethod] fn dmh3510()  -> Self { Self { inner: DMH3510  } }
    #[staticmethod] fn dmh6220()  -> Self { Self { inner: DMH6220  } }
    #[staticmethod] fn dmjh11()   -> Self { Self { inner: DMJH11   } }
}

// ---------------------------------------------------------------------------
// MitSetpoint
// ---------------------------------------------------------------------------

#[gen_stub_pyclass]
#[pyclass(name = "MitSetpoint", module = "motorcom")]
#[derive(Clone, Copy)]
pub struct PyMitSetpoint {
    pub(crate) inner: MitSetpoint,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyMitSetpoint {
    #[new]
    #[pyo3(signature = (q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0))]
    fn new(q: f32, dq: f32, kp: f32, kd: f32, tau: f32) -> Self {
        Self { inner: MitSetpoint { q, dq, kp, kd, tau } }
    }

    #[getter] fn q(&self)   -> f32 { self.inner.q   }
    #[getter] fn dq(&self)  -> f32 { self.inner.dq  }
    #[getter] fn kp(&self)  -> f32 { self.inner.kp  }
    #[getter] fn kd(&self)  -> f32 { self.inner.kd  }
    #[getter] fn tau(&self) -> f32 { self.inner.tau }

    fn __repr__(&self) -> String {
        let s = self.inner;
        format!("MitSetpoint(q={}, dq={}, kp={}, kd={}, tau={})", s.q, s.dq, s.kp, s.kd, s.tau)
    }
}

// ---------------------------------------------------------------------------
// MotorState
// ---------------------------------------------------------------------------

#[gen_stub_pyclass]
#[pyclass(name = "MotorState", module = "motorcom")]
#[derive(Clone, Copy)]
pub struct PyMotorState {
    pub(crate) inner: MotorState,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyMotorState {
    #[getter] fn motor_id(&self) -> u8  { self.inner.motor_id }
    #[getter] fn error(&self)    -> u8  { self.inner.error    }
    #[getter] fn position(&self) -> f32 { self.inner.position }
    #[getter] fn velocity(&self) -> f32 { self.inner.velocity }
    #[getter] fn torque(&self)   -> f32 { self.inner.torque   }
    #[getter] fn t_mos(&self)    -> u8  { self.inner.t_mos    }
    #[getter] fn t_rotor(&self)  -> u8  { self.inner.t_rotor  }

    fn __repr__(&self) -> String {
        let s = self.inner;
        format!(
            "MotorState(motor_id={}, error={}, position={:.4}, velocity={:.4}, \
             torque={:.4}, t_mos={}, t_rotor={})",
            s.motor_id, s.error, s.position, s.velocity, s.torque, s.t_mos, s.t_rotor
        )
    }
}

// ---------------------------------------------------------------------------
// Transport — wrapped behind a Mutex so Python's GIL-released calls are sound.
// ---------------------------------------------------------------------------

#[cfg(feature = "socketcan-backend")]
#[gen_stub_pyclass]
#[pyclass(name = "SocketCanTransport", module = "motorcom")]
pub struct PySocketCanTransport {
    pub(crate) inner: Mutex<SocketCanTransport>,
}

#[cfg(feature = "socketcan-backend")]
#[gen_stub_pymethods]
#[pymethods]
impl PySocketCanTransport {
    /// Open a SocketCAN interface (e.g. `"can0"`). Bring it up first with
    /// `sudo ip link set can0 up type can bitrate 1000000`.
    #[new]
    fn new(iface: &str) -> PyResult<Self> {
        let inner = SocketCanTransport::open(iface).map_err(can_err_to_py)?;
        Ok(Self { inner: Mutex::new(inner) })
    }
}

// ---------------------------------------------------------------------------
// Damiao
// ---------------------------------------------------------------------------

#[gen_stub_pyclass]
#[pyclass(name = "Damiao", module = "motorcom")]
pub struct PyDamiao {
    inner: Damiao,
}

// Always-on PyDamiao methods: constructors, getters, repr. Available even
// when no transport backend is compiled in (e.g. wheel-only data classes
// on Windows / macOS).
#[gen_stub_pymethods]
#[pymethods]
impl PyDamiao {
    /// `motor_id` is the id the motor *listens* on; `master_id` is the id it
    /// *replies* on. Optionally pass per-variant `MitLimits`.
    #[new]
    #[pyo3(signature = (motor_id, master_id, limits=None, timeout_ms=20))]
    fn new(motor_id: u16, master_id: u16, limits: Option<PyMitLimits>, timeout_ms: u64) -> Self {
        let mut dm = Damiao::new(motor_id, master_id)
            .with_timeout(Duration::from_millis(timeout_ms));
        if let Some(l) = limits {
            dm = dm.with_limits(l.inner);
        }
        Self { inner: dm }
    }

    #[getter] fn motor_id(&self)   -> u16 { self.inner.motor_id  }
    #[getter] fn master_id(&self)  -> u16 { self.inner.master_id }

    fn __repr__(&self) -> String {
        format!(
            "Damiao(motor_id={:#x}, master_id={:#x})",
            self.inner.motor_id, self.inner.master_id
        )
    }
}

// CAN ops gated on the SocketCAN backend. Putting them in a separate
// `impl` block (rather than `#[cfg]`-gating individual methods) keeps
// `gen_stub_pymethods` from emitting metadata that references
// `PySocketCanTransport` when the type doesn't exist.
//
// CAN ops use a short per-command timeout (~20 ms default), so we keep
// the GIL across the call rather than mediating Send-ness for the
// SocketCanTransport handle. `bus.borrow()` returns a `PyRef` that
// must out-live the lock guard, so each call binds it explicitly.
#[cfg(feature = "socketcan-backend")]
#[gen_stub_pymethods]
#[pymethods]
impl PyDamiao {
    fn enable(&self, bus: &Bound<'_, PySocketCanTransport>) -> PyResult<PyMotorState> {
        let bus_ref = bus.borrow();
        let mut guard = bus_ref.inner.lock().unwrap();
        self.inner
            .enable(&mut *guard as &mut dyn CanTransport)
            .map(|s| PyMotorState { inner: s })
            .map_err(dm_err_to_py)
    }

    fn disable(&self, bus: &Bound<'_, PySocketCanTransport>) -> PyResult<PyMotorState> {
        let bus_ref = bus.borrow();
        let mut guard = bus_ref.inner.lock().unwrap();
        self.inner
            .disable(&mut *guard as &mut dyn CanTransport)
            .map(|s| PyMotorState { inner: s })
            .map_err(dm_err_to_py)
    }

    fn set_zero(&self, bus: &Bound<'_, PySocketCanTransport>) -> PyResult<PyMotorState> {
        let bus_ref = bus.borrow();
        let mut guard = bus_ref.inner.lock().unwrap();
        self.inner
            .set_zero(&mut *guard as &mut dyn CanTransport)
            .map(|s| PyMotorState { inner: s })
            .map_err(dm_err_to_py)
    }

    fn clear_error(&self, bus: &Bound<'_, PySocketCanTransport>) -> PyResult<PyMotorState> {
        let bus_ref = bus.borrow();
        let mut guard = bus_ref.inner.lock().unwrap();
        self.inner
            .clear_error(&mut *guard as &mut dyn CanTransport)
            .map(|s| PyMotorState { inner: s })
            .map_err(dm_err_to_py)
    }

    fn mit_control(
        &self,
        bus: &Bound<'_, PySocketCanTransport>,
        setpoint: PyMitSetpoint,
    ) -> PyResult<PyMotorState> {
        let bus_ref = bus.borrow();
        let mut guard = bus_ref.inner.lock().unwrap();
        self.inner
            .mit_control(&mut *guard as &mut dyn CanTransport, &setpoint.inner)
            .map(|s| PyMotorState { inner: s })
            .map_err(dm_err_to_py)
    }

    /// Force the firmware into MIT control mode (register 10 = 1). Some
    /// motors ship in POS_VEL/VEL/FORCE_POS, in which case `mit_control` is
    /// silently ignored.
    fn ensure_mit_mode(&self, bus: &Bound<'_, PySocketCanTransport>) -> PyResult<()> {
        let bus_ref = bus.borrow();
        let mut guard = bus_ref.inner.lock().unwrap();
        self.inner
            .ensure_control_mode(&mut *guard as &mut dyn CanTransport, ControlMode::Mit)
            .map_err(dm_err_to_py)
    }
}

// ---------------------------------------------------------------------------
// Free helpers
// ---------------------------------------------------------------------------

/// Build a `MitSetpoint` (handy when keyword args are awkward).
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0))]
fn mit_setpoint(q: f32, dq: f32, kp: f32, kd: f32, tau: f32) -> PyMitSetpoint {
    PyMitSetpoint { inner: MitSetpoint { q, dq, kp, kd, tau } }
}

// ---------------------------------------------------------------------------
// Module entry point
// ---------------------------------------------------------------------------

#[pymodule]
fn motorcom(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Forward `log::info!` / `log::warn!` / etc. from Rust code into
    // Python's `logging` module. Configure it on the Python side with
    // `logging.basicConfig(level=logging.DEBUG)` to see the output.
    pyo3_log::init();

    m.add("__version__", env!("CARGO_PKG_VERSION"))?;

    m.add_class::<PyMitLimits>()?;
    m.add_class::<PyMitSetpoint>()?;
    m.add_class::<PyMotorState>()?;
    m.add_class::<PyDamiao>()?;

    #[cfg(feature = "socketcan-backend")]
    m.add_class::<PySocketCanTransport>()?;

    m.add_function(wrap_pyfunction!(mit_setpoint, m)?)?;
    Ok(())
}

// Stub-info gatherer used by `src/bin/stub_gen.rs` to emit `.pyi` files.
define_stub_info_gatherer!(stub_info);
