use std::sync::Arc;
use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, log_info};

/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("param_declare")?;
    let logger = node.logger().clone();
    log_info!(&logger, "Hello from learning_parameter package!");

    let robot_name_param = node.declare_parameter::<Arc<str>>("robot_name")
        .default("mbot".into())
        .description("The name of the robot")
        .mandatory()
        .unwrap();

    log_info!(&logger, "Declared parameter: {}", robot_name_param.get());

    let timer_option = rclrs::TimerOptions::new(std::time::Duration::from_secs(1));

    let _timer = node.create_timer_repeating(timer_option, move || {
        log_info!(&logger, "Current robot name parameter: {}", robot_name_param.get());
        robot_name_param.set(Arc::from("new_robot_name")).unwrap();
        log_info!(&logger, "Updated robot name parameter to: {}", robot_name_param.get());
    })?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
