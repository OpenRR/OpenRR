use std::{sync::Arc, time::Duration};

use crate::{collision::parse_colon_separated_pairs, CollisionChecker};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::debug;

use crate::utils::find_nodes;

pub struct SelfCollisionChecker {
    pub using_joints: k::Chain<f64>,
    pub collision_check_robot: Arc<k::Chain<f64>>,
    pub collision_checker: CollisionChecker<f64>,
    pub collision_pairs: Vec<(String, String)>,
    pub time_interpolate_rate: f64,
}

impl SelfCollisionChecker {
    #[track_caller]
    pub fn new(
        joint_names: Vec<String>,
        collision_check_robot: Arc<k::Chain<f64>>,
        collision_checker: CollisionChecker<f64>,
        collision_pairs: Vec<(String, String)>,
        time_interpolate_rate: f64,
    ) -> Self {
        assert!(
            time_interpolate_rate > 0.0 && time_interpolate_rate <= 1.0,
            "time_interpolate_rate must be 0.0~1.0 but {}",
            time_interpolate_rate
        );
        let using_joints =
            k::Chain::<f64>::from_nodes(find_nodes(&joint_names, &collision_check_robot).unwrap());
        Self {
            using_joints,
            collision_check_robot,
            collision_checker,
            collision_pairs,
            time_interpolate_rate,
        }
    }

    pub fn check_joint_positions(
        &self,
        current: &[f64],
        positions: &[f64],
        duration: std::time::Duration,
    ) -> Result<(), Error> {
        match interpolate(
            &[current.to_vec(), positions.to_vec()],
            duration.as_secs_f64(),
            duration.as_secs_f64() * self.time_interpolate_rate,
        ) {
            Some(interpolated) => {
                debug!("interpolated len={}", interpolated.len());
                for v in interpolated {
                    self.using_joints.set_joint_positions_clamped(&v.position);
                    self.collision_check_robot.update_transforms();
                    let mut self_checker = self
                        .collision_checker
                        .check_self(&self.collision_check_robot, &self.collision_pairs);
                    if let Some(names) = self_checker.next() {
                        return Err(Error::CollisionError(names.0, names.1));
                    }
                    let mut vec_used: Vec<_> = self_checker.used_duration().iter().collect();
                    vec_used.sort_by(|a, b| b.1.cmp(a.1));
                    let sum_duration: Duration =
                        self_checker.used_duration().iter().map(|(_k, v)| v).sum();
                    debug!("total: {:?}", sum_duration);
                    debug!("detailed: {:?}", vec_used);
                }
                Ok(())
            }
            None => Err(Error::InterpolationError(
                "failed to interpolate".to_owned(),
            )),
        }
    }

    pub fn check_joint_trajectory(&self, trajectory: &[TrajectoryPoint]) -> Result<(), Error> {
        for v in trajectory {
            self.using_joints
                .set_joint_positions(&v.positions)
                .map_err(|e| Error::Other(e.into()))?;
            if let Some(names) = self
                .collision_checker
                .check_self(&self.collision_check_robot, &self.collision_pairs)
                .next()
            {
                return Err(Error::CollisionError(names.0, names.1));
            }
        }
        Ok(())
    }
}
