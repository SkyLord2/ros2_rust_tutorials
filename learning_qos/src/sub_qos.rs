use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, SubscriptionOptions, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, log_info};
use std_msgs::msg::String as StringMsg;

fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("pub_qos")?;
    let logger = node.logger().clone();
    log_info!(&logger, "ROS2节点示例: 服务质量");

    let mut subscription_options = SubscriptionOptions::new("hello_world_qos");

    let mut qos = QoSProfile::default();
    qos.history = QoSHistoryPolicy::KeepLast { depth: 20 };
    qos.reliability = QoSReliabilityPolicy::Reliable;
    subscription_options.qos = qos;

    let _subscription = node.create_subscription::<StringMsg, _>(
        subscription_options,
        move |msg: StringMsg| {
            log_info!(&logger, "Received message: {}", msg.data);
        }
    )?;


    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}