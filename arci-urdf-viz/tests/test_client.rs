mod web_server;

use std::time::Duration;

use arci::{BaseVelocity, JointTrajectoryClient, MoveBase, TrajectoryPoint};
use arci_urdf_viz::{UrdfVizWebClient, UrdfVizWebClientConfig};
use assert_approx_eq::assert_approx_eq;
use url::Url;
use web_server::*;

#[test]
fn test_urdf_viz_web_client_config_accessor() {
    let mut config = UrdfVizWebClientConfig {
        name: "test".to_owned(),
        joint_names: Some(vec!["j1".to_owned(), "j2".to_owned()]),
        wrap_with_joint_position_limiter: true,
        joint_position_limits: None,
        wrap_with_joint_velocity_limiter: true,
        joint_velocity_limits: Some(vec![1.0, 2.0]),
    };
    assert_eq!(config.name, "test");
    assert_eq!(config.joint_names.as_ref().unwrap()[0], "j1");
    assert_eq!(config.joint_names.as_ref().unwrap()[1], "j2");
    assert!(config.wrap_with_joint_position_limiter);
    assert!(config.joint_position_limits.is_none());
    assert!(config.wrap_with_joint_velocity_limiter);
    assert_approx_eq!(config.joint_velocity_limits.as_ref().unwrap()[0], 1.0);
    assert_approx_eq!(config.joint_velocity_limits.unwrap()[1], 2.0);
    config.name = "arm".to_owned();
    config.joint_names = Some(vec![
        "shoulder_pan_joint".to_owned(),
        "elbow_joint".to_owned(),
    ]);
    config.wrap_with_joint_position_limiter = false;
    config.joint_position_limits = Some(vec![]);
    config.wrap_with_joint_velocity_limiter = false;
    config.joint_velocity_limits = Some(vec![0.0, 0.0]);
    assert_eq!(config.name, "arm");
    assert_eq!(
        config.joint_names.as_ref().unwrap()[0],
        "shoulder_pan_joint"
    );
    assert_eq!(config.joint_names.as_ref().unwrap()[1], "elbow_joint");
    assert!(!config.wrap_with_joint_position_limiter);
    assert!(config.joint_position_limits.unwrap().is_empty());
    assert!(!config.wrap_with_joint_velocity_limiter);
    assert_approx_eq!(config.joint_velocity_limits.as_ref().unwrap()[0], 0.0);
    assert_approx_eq!(config.joint_velocity_limits.unwrap()[1], 0.0);
}

#[test]
fn test_urdf_viz_web_client_config_debug() {
    let config = UrdfVizWebClientConfig {
        name: "test".to_owned(),
        joint_names: Some(vec!["j1".to_owned(), "j2".to_owned()]),
        wrap_with_joint_position_limiter: true,
        joint_position_limits: None,
        wrap_with_joint_velocity_limiter: true,
        joint_velocity_limits: Some(vec![1.0, 2.0]),
    };
    assert_eq!(
        format!("{:?}", config),
        "UrdfVizWebClientConfig { name: \"test\", \
            joint_names: Some([\"j1\", \"j2\"]), \
            wrap_with_joint_position_limiter: true, \
            wrap_with_joint_velocity_limiter: true, \
            joint_velocity_limits: Some([1.0, 2.0]), \
            joint_position_limits: None \
        }"
    )
}

#[allow(clippy::redundant_clone)] // This is intentional.
#[test]
fn test_urdf_viz_web_client_config_clone() {
    let config1 = UrdfVizWebClientConfig {
        name: "test".to_owned(),
        joint_names: Some(vec!["j1".to_owned(), "j2".to_owned()]),
        wrap_with_joint_position_limiter: true,
        joint_position_limits: None,
        wrap_with_joint_velocity_limiter: true,
        joint_velocity_limits: Some(vec![1.0, 2.0]),
    };
    let config2 = config1.clone();
    assert_eq!(config2.name, "test");
    assert_eq!(config2.joint_names.as_ref().unwrap()[0], "j1");
    assert_eq!(config2.joint_names.as_ref().unwrap()[1], "j2");
    assert!(config2.wrap_with_joint_position_limiter);
    assert!(config2.joint_position_limits.is_none());
    assert!(config2.wrap_with_joint_velocity_limiter);
    assert_approx_eq!(config2.joint_velocity_limits.as_ref().unwrap()[0], 1.0);
    assert_approx_eq!(config2.joint_velocity_limits.unwrap()[1], 2.0);
}

#[test]
fn test_create_joint_trajectory_clients() {
    const DEFAULT_PORT: u16 = 7777;
    let web_server = WebServer::new(DEFAULT_PORT);
    web_server.set_current_joint_positions(JointNamesAndPositions {
        names: vec!["j1".to_owned(), "j2".to_owned()],
        positions: vec![1.0, -1.0],
    });
    web_server.start_background();
    let configs = vec![
        UrdfVizWebClientConfig {
            name: "c1".to_owned(),
            joint_names: Some(vec!["j1".to_owned(), "j2".to_owned()]),
            wrap_with_joint_position_limiter: false,
            joint_position_limits: None,
            wrap_with_joint_velocity_limiter: true,
            joint_velocity_limits: Some(vec![1.0, 1.0]),
        },
        UrdfVizWebClientConfig {
            name: "c2".to_owned(),
            joint_names: Some(vec!["j1".to_owned(), "j2".to_owned()]),
            wrap_with_joint_position_limiter: false,
            joint_position_limits: None,
            wrap_with_joint_velocity_limiter: false,
            joint_velocity_limits: None,
        },
    ];
    let _clients = arci_urdf_viz::create_joint_trajectory_clients(configs, None).unwrap();
}

#[test]
fn test_current_joint_positions() {
    const PORT: u16 = 7778;
    let web_server = WebServer::new(PORT);
    web_server.set_current_joint_positions(JointNamesAndPositions {
        names: vec!["j1".to_owned(), "j2".to_owned()],
        positions: vec![1.0, -1.0],
    });
    web_server.start_background();
    let c =
        UrdfVizWebClient::new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap()).unwrap();
    let v = c.current_joint_positions().unwrap();
    assert_approx_eq!(v[0], 1.0);
    assert_approx_eq!(v[1], -1.0);
}

#[tokio::test]
async fn test_send_joint_positions() {
    const PORT: u16 = 7780;
    let web_server = WebServer::new(PORT);
    web_server.set_current_joint_positions(JointNamesAndPositions {
        names: vec!["j1".to_owned()],
        positions: vec![0.0],
    });
    web_server.start_background();
    let client =
        UrdfVizWebClient::new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap()).unwrap();
    client.run_send_joint_positions_thread();
    let result = client
        .send_joint_positions(vec![1.0], Duration::from_secs(1))
        .unwrap()
        .await;
    assert!(result.is_ok());
    let v = client.current_joint_positions().unwrap();
    assert_approx_eq!(v[0], 1.0);
}

#[test]
fn test_send_joint_positions_no_wait() {
    const PORT: u16 = 7781;
    let web_server = WebServer::new(PORT);
    web_server.set_current_joint_positions(JointNamesAndPositions {
        names: vec!["j1".to_owned()],
        positions: vec![0.0],
    });
    web_server.start_background();
    let client =
        UrdfVizWebClient::new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap()).unwrap();
    client.run_send_joint_positions_thread();
    let _ = client
        .send_joint_positions(vec![1.0], Duration::from_secs(1))
        .unwrap();
    let v = client.current_joint_positions().unwrap();
    assert_approx_eq!(v[0], 0.0);
    std::thread::sleep(Duration::from_secs(2));
    let v = client.current_joint_positions().unwrap();
    assert_approx_eq!(v[0], 1.0);
}

#[tokio::test]
async fn test_send_joint_trajectory() {
    const PORT: u16 = 7782;
    let web_server = WebServer::new(PORT);
    web_server.set_current_joint_positions(JointNamesAndPositions {
        names: vec!["j1".to_owned()],
        positions: vec![0.0],
    });
    web_server.start_background();
    let client =
        UrdfVizWebClient::new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap()).unwrap();
    client.run_send_joint_positions_thread();

    let trajectory = vec![
        TrajectoryPoint::new(vec![1.0], Duration::from_millis(100)),
        TrajectoryPoint::new(vec![2.0], Duration::from_millis(200)),
    ];
    client
        .send_joint_trajectory(trajectory)
        .unwrap()
        .await
        .unwrap();
    let v = client.current_joint_positions().unwrap();
    assert_approx_eq!(v[0], 2.0);

    let trajectory = vec![
        TrajectoryPoint::new(vec![1.0], Duration::from_millis(1)),
        TrajectoryPoint::new(vec![2.0], Duration::from_millis(2)),
        TrajectoryPoint::new(vec![3.0], Duration::from_millis(3)),
        TrajectoryPoint::new(vec![4.0], Duration::from_millis(4)),
        TrajectoryPoint::new(vec![5.0], Duration::from_millis(5)),
    ];
    client
        .send_joint_trajectory(trajectory)
        .unwrap()
        .await
        .unwrap();
    let v = client.current_joint_positions().unwrap();
    assert_approx_eq!(v[0], 5.0);
}

#[test]
#[should_panic = "send_joint_positions called without run_send_joint_positions_thread being called first"]
fn send_joint_positions_without_send_joint_positions_thread() {
    const PORT: u16 = 7783;
    let web_server = WebServer::new(PORT);
    web_server.set_current_joint_positions(JointNamesAndPositions {
        names: vec!["j1".to_owned()],
        positions: vec![0.0],
    });
    web_server.start_background();
    let client =
        UrdfVizWebClient::new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap()).unwrap();
    let _ = client
        .send_joint_positions(vec![1.0], Duration::from_secs(1))
        .unwrap();
}

#[test]
#[should_panic = "send_velocity called without run_send_velocity_thread being called first"]
fn send_velocity_without_send_velocity_thread() {
    const PORT: u16 = 7784;
    let web_server = WebServer::new(PORT);
    web_server.start_background();
    let client =
        UrdfVizWebClient::new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap()).unwrap();
    client.send_velocity(&BaseVelocity::default()).unwrap();
}
