//! Rust driver for an iRobot Roomba/Create over the serial Open Interface.
//!
//! Protocol behaviour is ported from the reference Python `create.py`.

pub mod protocol;
pub mod robot;
pub mod sensors;
