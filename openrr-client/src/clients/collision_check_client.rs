use std::{path::Path, sync::Arc, time::Duration};

use arci::{Error, JointTrajectoryClient, TrajectoryPoint, WaitFuture};
use openrr_planner::{collision::parse_colon_separated_pairs, CollisionChecker};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::debug;

use crate::utils::find_nodes;

pub struct CollisionCheckClient<T>
where
    T: JointTrajectoryClient,
{
    pub client: T,
    pub collision_checker: Arc<SelfCollisionChecker>,
}

impl<T> CollisionCheckClient<T>
where
    T: JointTrajectoryClient,
{
    pub fn new(client: T, collision_checker: Arc<SelfCollisionChecker>) -> Self {
        Self {
            client,
            collision_checker,
        }
    }
}

impl<T> JointTrajectoryClient for CollisionCheckClient<T>
where
    T: JointTrajectoryClient,
{
    fn joint_names(&self) -> Vec<String> {
        self.client.joint_names()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        self.client.current_joint_positions()
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        self.collision_checker.check_joint_positions(
            &self.current_joint_positions()?,
            &positions,
            duration,
        )?;
        self.client.send_joint_positions(positions, duration)
    }

    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        self.collision_checker.check_joint_trajectory(&trajectory)?;
        self.client.send_joint_trajectory(trajectory)
    }
}

#[derive(Clone, Serialize, Deserialize, Debug, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SelfCollisionCheckerConfig {
    #[serde(default = "default_prediction")]
    pub prediction: f64,
    #[serde(default = "default_time_interpolate_rate")]
    pub time_interpolate_rate: f64,
}

fn default_prediction() -> f64 {
    0.001
}
fn default_time_interpolate_rate() -> f64 {
    0.5
}

impl Default for SelfCollisionCheckerConfig {
    fn default() -> Self {
        Self {
            prediction: default_prediction(),
            time_interpolate_rate: default_time_interpolate_rate(),
        }
    }
}

pub fn create_collision_check_client<P: AsRef<Path>>(
    urdf_path: P,
    self_collision_check_pairs: &[String],
    config: &SelfCollisionCheckerConfig,
    client: Arc<dyn JointTrajectoryClient>,
    full_chain: Arc<k::Chain<f64>>,
) -> CollisionCheckClient<Arc<dyn JointTrajectoryClient>> {
    let joint_names = client.joint_names();
    CollisionCheckClient::new(
        client,
        Arc::new(create_self_collision_checker(
            urdf_path,
            self_collision_check_pairs,
            joint_names,
            &config,
            full_chain,
        )),
    )
}

pub fn create_self_collision_checker<P: AsRef<Path>>(
    urdf_path: P,
    self_collision_check_pairs: &[String],
    joint_names: Vec<String>,
    config: &SelfCollisionCheckerConfig,
    full_chain: Arc<k::Chain<f64>>,
) -> SelfCollisionChecker {
    SelfCollisionChecker::new(
        joint_names,
        full_chain,
        CollisionChecker::from_urdf_robot(
            &urdf_rs::utils::read_urdf_or_xacro(urdf_path).unwrap(),
            config.prediction,
        ),
        parse_colon_separated_pairs(self_collision_check_pairs).unwrap(),
        config.time_interpolate_rate,
    )
}
