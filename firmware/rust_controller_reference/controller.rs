use super::estimator::Data;
use crate::G;
use core::hint::{assert_unchecked, cold_path};
use libm::log1pf;

const VELOCITY_CUTOFF: f32 = 5.0;
const MAX_CHANGE_PER_SECOND: f32 = 1.0;
const DECAY_RANGE: f32 = 7.5; // This is in meters.
const KP_PER_SECOND: f32 = MAX_CHANGE_PER_SECOND / DECAY_RANGE;

const DEPLOYMENT_MIN: f32 = 0.0;
const DEPLOYMENT_MAX: f32 = 1.0;

/// Airbrake control system.
///
/// The deployment of this controller will always be kept between `0.0` and `1.0`.
#[derive(Debug)]
pub struct Controller {
    deployment: f32,
    target_apogee: f32,
}

impl Controller {
    /// Creates a new [`Controller`] with a deployment of `0.0` and using `target_apogee`.
    ///
    /// The value of `target_apogee` must be **metric**.
    #[inline]
    pub fn new(target_apogee: f32) -> Self {
        let deployment = DEPLOYMENT_MIN;
        Self {
            deployment,
            target_apogee,
        }
    }

    /// Gets the current deployment of the airbrake control system.
    #[inline]
    pub fn deployment(&self) -> f32 {
        self.deployment
    }

    /// Gets the target apogee of the airbrake control system.
    #[inline]
    pub fn target_apogee(&self) -> f32 {
        self.target_apogee
    }

    /// Moves the airbrakes towards being closed.
    #[inline]
    pub fn close(&mut self, dt_s: f32) {
        let max_change = MAX_CHANGE_PER_SECOND * dt_s;
        self.deployment = (self.deployment - max_change).max(DEPLOYMENT_MIN);
    }

    /// Calculates the new deployment of the airbrakes, updates the current deployment,
    /// and returns that updated deployment value.
    ///
    /// The values of `altitude`, `vert_accel`, and `vert_velo` must all be **metric** and have finite values.
    ///
    /// The value of `dt_s` must be both positive and finite.
    #[inline(never)]
    pub fn new_deployment(&mut self, positional_data: Data, dt_s: f32) -> f32 {
        self.deployment = self
            .new_deployment_internal(positional_data, dt_s)
            .clamp(DEPLOYMENT_MIN, DEPLOYMENT_MAX);
        self.deployment
    }

    /// Algorithm derived from the formula for 1D motion of a coasting rocket.
    #[inline(always)]
    fn new_deployment_internal(&self, positional_data: Data, dt_s: f32) -> f32 {
        // Need to do this here for potential use in early return branch.
        let max_change = MAX_CHANGE_PER_SECOND * dt_s;
        // SAFETY: The value of `max_change` will always be positive,
        // so the value of `-max_change` will always be negative.
        unsafe {
            // Required to eliminate the panic branch from `clamp`.
            assert_unchecked(-max_change <= max_change);
        }

        // Starts airbrake de-deployment when close to apogee.
        if positional_data.vertical_velocity <= VELOCITY_CUTOFF {
            cold_path();
            return self.deployment - max_change;
        }

        // Closed form solution. Very cool.
        let vert_velo_2 = positional_data.vertical_velocity * positional_data.vertical_velocity;
        let k = -(positional_data.vertical_acceleration + G) / vert_velo_2;
        let delta_h = if k > 0.0 {
            let x = k * vert_velo_2 / G;
            log1pf(x) / (k + k)
        } else {
            // Unlikely to reach this fallback path except when getting close to apogee.
            cold_path();
            vert_velo_2 / (G + G)
        };

        let predicted_apogee = positional_data.altitude_agl + delta_h;
        let apogee_error = predicted_apogee - self.target_apogee;

        let desired_change = apogee_error * KP_PER_SECOND * dt_s;
        let actual_change = desired_change.clamp(-max_change, max_change);

        self.deployment + actual_change
    }
}
