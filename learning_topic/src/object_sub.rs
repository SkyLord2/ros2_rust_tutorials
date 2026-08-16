use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, log_info};
use learning_interface::msg::ObjectPosition as ObjectPositionMsg;

/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("object_sub")?;
    let logger = node.logger().clone();
    log_info!(&logger, "ROS2节点示例：订阅节点");

    let _subscription = node.create_subscription::<ObjectPositionMsg, _>(
        "object_position",
        move |msg: ObjectPositionMsg| {
            log_info!(&logger, "收到物体位置消息: ({}, {})", msg.x, msg.y);
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
