use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, log_info};
use std_msgs::msg::String as StringMsg;

/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("node_sub")?;
    let logger = node.logger().clone();
    log_info!(&logger, "ROS2节点示例：订阅节点");

    let _subscription = node.create_subscription::<StringMsg, _>(
        "hello_world",
        move |msg: StringMsg| {
            log_info!(&logger, "收到消息: {}", msg.data);
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
