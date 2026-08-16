use std::time::Duration;
use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, log_info};
use learning_interface::action::{MoveCircle, MoveCircle_Feedback, MoveCircle_Result};
/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("action_move_server")?;
    let logger = node.logger().clone();
    log_info!(&logger, "Hello from learning_action package!");

    let _action_server = node.create_action_server::<MoveCircle, _>("move_circle", move |requested_goal| {
        let logger = logger.clone();
        async move {
            log_info!(&logger, "Moving circle...");
            let mut feedback_msg = MoveCircle_Feedback::default();
            // 在发布反馈之前，目标必须先被接受并进入执行状态
            let goal_handle = requested_goal.accept().execute();
            // 从0到360度，执行圆周运动，并周期反馈信息
            for i in (0..=360).step_by(30) {
                // 创建反馈信息，表示当前执行到的角度
                feedback_msg.state = i as i32;
                log_info!(&logger, "Publishing feedback: {}", feedback_msg.state);
                
                // 发布反馈信息
                goal_handle.publish_feedback(feedback_msg.clone());

                
                // 休眠 0.5 秒
                std::thread::sleep(Duration::from_millis(500));
            }

            // 创建结果消息
            let mut result = MoveCircle_Result::default();
            result.finish = true;
            
            // The callback must return the terminated goal to the action server.
            goal_handle.succeeded_with(result)
        }
    })?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
