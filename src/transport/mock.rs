//! In-process loopback transport for tests.
//!
//! The two ends are two [`MockTransport`] instances sharing a pair of queues:
//! whatever side A sends, side B receives, and vice versa.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::{CanError, CanFrame, CanTransport};

#[derive(Default)]
struct Queue {
    frames: VecDeque<CanFrame>,
}

#[derive(Clone)]
pub struct MockTransport {
    inbox: Arc<Mutex<Queue>>,
    outbox: Arc<Mutex<Queue>>,
}

impl MockTransport {
    /// Build a connected pair `(a, b)`. Frames sent on `a` are received on `b`.
    pub fn pair() -> (Self, Self) {
        let q1 = Arc::new(Mutex::new(Queue::default()));
        let q2 = Arc::new(Mutex::new(Queue::default()));
        let a = MockTransport { inbox: q1.clone(), outbox: q2.clone() };
        let b = MockTransport { inbox: q2, outbox: q1 };
        (a, b)
    }

    /// Inspect the next frame waiting in this end's inbox without removing it.
    pub fn peek(&self) -> Option<CanFrame> {
        self.inbox.lock().unwrap().frames.front().cloned()
    }
}

impl CanTransport for MockTransport {
    fn send(&mut self, frame: &CanFrame) -> Result<(), CanError> {
        self.outbox.lock().unwrap().frames.push_back(frame.clone());
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> Result<CanFrame, CanError> {
        let deadline = std::time::Instant::now() + timeout;
        loop {
            if let Some(f) = self.inbox.lock().unwrap().frames.pop_front() {
                return Ok(f);
            }
            if std::time::Instant::now() >= deadline {
                return Err(CanError::Timeout);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::CanId;

    #[test]
    fn loopback_roundtrip() {
        let (mut a, mut b) = MockTransport::pair();
        let tx = CanFrame::classic(CanId::standard(0x10), vec![1, 2, 3]);
        a.send(&tx).unwrap();
        let rx = b.recv(Duration::from_millis(50)).unwrap();
        assert_eq!(rx.id, CanId::Standard(0x10));
        assert_eq!(rx.data, vec![1, 2, 3]);
    }

    #[test]
    fn recv_times_out_when_empty() {
        let (mut a, _b) = MockTransport::pair();
        assert!(matches!(
            a.recv(Duration::from_millis(5)),
            Err(CanError::Timeout)
        ));
    }
}
