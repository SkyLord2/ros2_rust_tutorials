use std::time::Duration;
use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, log_info};
use std_msgs::msg::String as StringMsg;

/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("node_pub")?;
    let logger = node.logger();
    log_info!(logger, "ROS2节点示例：发布节点");
    let publisher = node.create_publisher::<StringMsg>("hello_world")?;
    let options = rclrs::TimerOptions::new(Duration::from_secs(1));
    let _timer = node.create_timer_repeating(options, move || {
        let msg = StringMsg {
            data: "Hello, ROS2 from Rust!".to_string(),
        };
        publisher.publish(msg).unwrap();
    })?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
