use crate::{
    error::Error,
    traits::{JointTrajectoryClient, TrajectoryPoint},
    waits::WaitFuture,
};

pub struct PartialJointTrajectoryClient<C>
where
    C: JointTrajectoryClient,
{
    joint_names: Vec<String>,
    shared_client: C,
    full_joint_names: Vec<String>,
}

pub fn copy_joint_positions(
    from_joint_names: &[String],
    from_positions: &[f64],
    to_joint_names: &[String],
    to_positions: &mut [f64],
) {
    for (to_index, to_joint_name) in to_joint_names.iter().enumerate() {
        if let Some(from_index) = from_joint_names.iter().position(|x| x == to_joint_name) {
            to_positions[to_index] = from_positions[from_index];
        }
    }
}

impl<C> PartialJointTrajectoryClient<C>
where
    C: JointTrajectoryClient,
{
    pub fn new(joint_names: Vec<String>, shared_client: C) -> Self {
        let full_joint_names = shared_client.joint_names().to_vec();
        Self {
            joint_names,
            shared_client,
            full_joint_names,
        }
    }
}

impl<C> JointTrajectoryClient for PartialJointTrajectoryClient<C>
where
    C: JointTrajectoryClient,
{
    fn joint_names(&self) -> Vec<String> {
        self.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        let mut result = vec![0.0; self.joint_names.len()];
        copy_joint_positions(
            &self.full_joint_names,
            &self.shared_client.current_joint_positions()?,
            &self.joint_names(),
            &mut result,
        );
        Ok(result)
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        let mut full_positions = self.shared_client.current_joint_positions()?;
        copy_joint_positions(
            &self.joint_names(),
            &positions,
            &self.full_joint_names,
            &mut full_positions,
        );
        self.shared_client
            .send_joint_positions(full_positions, duration)
    }

    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        let full_positions_base = self.shared_client.current_joint_positions()?;
        let mut full_trajectory = vec![];
        let full_dof = full_positions_base.len();
        for point in trajectory {
            let mut full_positions = full_positions_base.clone();
            copy_joint_positions(
                &self.joint_names(),
                &point.positions,
                &self.full_joint_names,
                &mut full_positions,
            );
            let mut full_point = TrajectoryPoint::new(full_positions, point.time_from_start);
            if let Some(partial_velocities) = &point.velocities {
                let mut full_velocities = vec![0.0; full_dof];
                copy_joint_positions(
                    &self.joint_names(),
                    &partial_velocities,
                    &self.full_joint_names,
                    &mut full_velocities,
                );
                full_point.velocities = Some(full_velocities);
            }
            full_trajectory.push(full_point);
        }
        self.shared_client.send_joint_trajectory(full_trajectory)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use assert_approx_eq::assert_approx_eq;

    use super::*;

    #[derive(Debug, Clone)]
    struct DummyFull {
        name: Vec<String>,
        pos: Arc<Mutex<Vec<f64>>>,
        last_trajectory: Arc<Mutex<Vec<TrajectoryPoint>>>,
    }
    impl JointTrajectoryClient for DummyFull {
        fn joint_names(&self) -> &[String] {
            &self.name
        }

        fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
            Ok(self.pos.lock().unwrap().clone())
        }

        fn send_joint_positions(
            &self,
            positions: Vec<f64>,
            _duration: std::time::Duration,
        ) -> Result<WaitFuture, Error> {
            *self.pos.lock().unwrap() = positions;
            Ok(WaitFuture::ready())
        }

        fn send_joint_trajectory(
            &self,
            full_trajectory: Vec<TrajectoryPoint>,
        ) -> Result<WaitFuture, Error> {
            if let Some(last_point) = full_trajectory.last() {
                *self.pos.lock().unwrap() = last_point.positions.to_owned();
            }
            *self.last_trajectory.lock().unwrap() = full_trajectory;
            Ok(WaitFuture::ready())
        }
    }

    #[test]
    fn test_partial_new() {
        let client = DummyFull {
            name: vec![String::from("part1"), String::from("high")],
            pos: Arc::new(Mutex::new(vec![1.0_f64, 3.0_f64])),
            last_trajectory: Arc::new(Mutex::new(Vec::new())),
        };
        let joint_names = vec![
            String::from("part1"),
            String::from("high"),
            String::from("part2"),
            String::from("low"),
        ];

        let partial = PartialJointTrajectoryClient::new(joint_names.clone(), client.clone());

        assert_eq!(
            format!("{:?}", partial.joint_names),
            "[\"part1\", \"high\", \"part2\", \"low\"]"
        );
        assert_eq!(
            format!("{:?}", client),
            format!("{:?}", partial.shared_client)
        );
        assert_eq!(
            format!("{:?}", partial.full_joint_names),
            "[\"part1\", \"high\"]"
        )
    }

    #[test]
    fn test_fn_copy_joint_position() {
        let from_joint_names = vec![
            String::from("part1"),
            String::from("part2"),
            String::from("part3"),
            String::from("part4"),
        ];
        let from_positions = vec![2.1_f64, 4.8, 1.0, 6.5];
        let to_joint_names = vec![
            String::from("part1"),
            String::from("part2"),
            String::from("part3"),
            String::from("part4"),
        ];
        let mut to_positions = vec![3.3_f64, 8.1, 5.2, 0.8];

        // lexical order pattern
        copy_joint_positions(
            &from_joint_names,
            &from_positions,
            &to_joint_names,
            &mut to_positions,
        );
        to_positions
            .iter()
            .zip(from_positions.iter())
            .for_each(|(pos, correct)| assert_approx_eq!(*pos, correct));
        println!("{:?}", to_positions);

        // random order pattern
        let to_joint_names = vec![
            String::from("part4"),
            String::from("part1"),
            String::from("part3"),
            String::from("part2"),
        ];
        let correct = vec![6.5_f64, 2.1, 1.0, 4.8];
        copy_joint_positions(
            &from_joint_names,
            &from_positions,
            &to_joint_names,
            &mut to_positions,
        );
        to_positions
            .iter()
            .zip(correct.iter())
            .for_each(|(pos, correct)| assert_approx_eq!(*pos, correct));
        println!("{:?}", to_positions);
    }

    #[test]
    fn test_partial_joint_name() {
        let client = DummyFull {
            name: vec![String::from("part1"), String::from("high")],
            pos: Arc::new(Mutex::new(vec![1.0_f64, 3.0_f64])),
            last_trajectory: Arc::new(Mutex::new(Vec::new())),
        };
        let joint_names = vec![
            String::from("part1"),
            String::from("high"),
            String::from("part2"),
            String::from("low"),
        ];

        let partial = PartialJointTrajectoryClient::new(joint_names.clone(), client.clone());

        assert_eq!(
            format!("{:?}", partial.joint_names()),
            "[\"part1\", \"high\", \"part2\", \"low\"]"
        );
    }

    #[test]
    fn test_partial_current_pos() {
        let client = DummyFull {
            name: vec![String::from("part1"), String::from("high")],
            pos: Arc::new(Mutex::new(vec![1.0_f64, 3.0_f64])),
            last_trajectory: Arc::new(Mutex::new(Vec::new())),
        };
        let joint_names = vec![
            String::from("part1"),
            String::from("high"),
            String::from("part2"),
            String::from("low"),
        ];
        let correct = vec![1.0_f64, 3.0_f64];

        let partial = PartialJointTrajectoryClient::new(joint_names.clone(), client.clone());
        let current_pos = partial.current_joint_positions();
        assert!(current_pos.is_ok());
        let current_pos = current_pos.unwrap();

        current_pos
            .iter()
            .zip(correct.iter())
            .for_each(|(pos, correct)| assert_approx_eq!(*pos, *correct));
    }

    #[tokio::test]
    async fn test_partial_send_pos() {
        let client = DummyFull {
            name: vec![String::from("part1"), String::from("high")],
            pos: Arc::new(Mutex::new(vec![1.0_f64, 3.0_f64])),
            last_trajectory: Arc::new(Mutex::new(Vec::new())),
        };
        let joint_names = vec![
            String::from("part1"),
            String::from("high"),
            String::from("part2"),
            String::from("low"),
        ];
        let next_pos = vec![2.2_f64, 0.5];
        let duration = std::time::Duration::from_secs(5);

        let partial = PartialJointTrajectoryClient::new(joint_names.clone(), client.clone());

        let result = partial.send_joint_positions(next_pos.clone(), duration);
        assert!(result.is_ok());
        assert!(result.unwrap().await.is_ok());

        let current_pos = partial.current_joint_positions().unwrap();
        current_pos
            .iter()
            .zip(next_pos.iter())
            .for_each(|(pos, correct)| assert_approx_eq!(*pos, *correct));
    }

    #[tokio::test]
    async fn test_partial_trajectory() {
        let client = DummyFull {
            name: vec![
                String::from("part1"),
                String::from("high"),
                String::from("part2"),
                String::from("low"),
            ],
            pos: Arc::new(Mutex::new(vec![5.1_f64, 0.8, 2.4, 4.5])),
            last_trajectory: Arc::new(Mutex::new(Vec::new())),
        };
        let joint_names = vec![
            String::from("part1"),
            String::from("high"),
            String::from("part2"),
            String::from("low"),
        ];
        let trajectories = vec![
            TrajectoryPoint::new(
                vec![1.0_f64, 3.0_f64, 2.2_f64, 4.0_f64],
                std::time::Duration::from_secs(1),
            ),
            TrajectoryPoint::new(
                vec![3.4_f64, 5.8_f64, 0.1_f64, 2.5_f64],
                std::time::Duration::from_secs(2),
            ),
        ];
        let correct = vec![3.4_f64, 5.8_f64, 0.1_f64, 2.5_f64];

        let partial = PartialJointTrajectoryClient::new(joint_names.clone(), client.clone());
        let result = partial.send_joint_trajectory(trajectories);
        assert!(result.is_ok());
        assert!(result.unwrap().await.is_ok());

        let current_pos = partial.current_joint_positions().unwrap();
        println!("{:?}", current_pos);
        current_pos
            .iter()
            .zip(correct.iter())
            .for_each(|(pos, correct)| assert_approx_eq!(*pos, *correct));
    }
}
