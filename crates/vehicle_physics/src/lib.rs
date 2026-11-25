//! Building blocks for deformable vehicles in Bevy.

#[cfg(feature = "avian")]
pub mod avian_suspension;
#[cfg(feature = "avian")]
pub mod avian_tyre;
pub mod beam_damage;
pub mod elastic_tyre;
pub mod kerb;
pub mod mounted_tyre;
pub mod pressure;
pub mod puncture;
pub mod tread_reinforcement;
pub mod triangle_road;
pub mod tyre;

pub mod inflation;
#[cfg(feature = "avian")]
pub mod reference_car;
