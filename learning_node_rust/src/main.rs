use anyhow::Result;
use std::time::Duration;
use std::thread;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, log_info};

/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("learning_node_rust_node")?;
    let logger = node.logger();
    logger.set_level(rclrs::LogSeverity::Info)?;
    while context.ok() {
        log_info!(logger, "Hello from learning_node_rust package!");
        thread::sleep(Duration::from_millis(500));
    }
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
