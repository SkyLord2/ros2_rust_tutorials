use std::{sync::atomic::{AtomicUsize, Ordering}};
use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, PublisherOptions, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, log_info};
use std_msgs::msg::String as StringMsg;

static PUBLISH_COUNT: AtomicUsize = AtomicUsize::new(0);
/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("pub_qos")?;
    let logger = node.logger().clone();
    log_info!(&logger, "ROS2节点示例: 服务质量");

    let mut publisher_options = PublisherOptions::from("hello_world_qos");
    let mut  qos = QoSProfile::default();
    qos.history = QoSHistoryPolicy::KeepLast { depth: 20 };
    qos.reliability = QoSReliabilityPolicy::Reliable;
    publisher_options.qos = qos;

    let publisher = node.create_publisher::<StringMsg>(publisher_options)?;

    let timer_options = rclrs::TimerOptions::new(std::time::Duration::from_secs(1));
    let _timer = node.create_timer_repeating(timer_options, move || {
        let old = PUBLISH_COUNT.fetch_add(1, Ordering::Relaxed);
        let msg = format!("Hello from pub_qos node! (#{})", old + 1);
        let log = format!("Publish message: {}", msg);
        log_info!(&logger, "{}", log);
        let msg = StringMsg { data: msg };
        publisher.publish(msg).unwrap();
    })?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
