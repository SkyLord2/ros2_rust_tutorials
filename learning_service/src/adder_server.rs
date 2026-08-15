use anyhow::Result;
use learning_interface::srv::{AddTwoInts, AddTwoInts_Request, AddTwoInts_Response};
use rclrs::{log_info, Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};

/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("adder_server")?;
    let logger = node.logger().clone();
    log_info!(&logger, "Hello from learning_service package!");

    let _server = node.create_service::<AddTwoInts, _>(
        "add_two_ints",
        move |request: AddTwoInts_Request| -> AddTwoInts_Response {
            let sum = request.a + request.b;
            log_info!(
                &logger,
                "Incoming request: a={}, b={}",
                request.a,
                request.b
            );
            AddTwoInts_Response { sum }
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
