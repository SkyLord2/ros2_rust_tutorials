use anyhow::Result;
use futures::StreamExt;
use rclrs::{Context, CreateBasicExecutor, GoalEvent, RclrsErrorFilter, SpinOptions, log_info};
use learning_interface::action::{MoveCircle, MoveCircle_Goal};

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    
    // 创建ROS2节点对象并进行初始化
    let node = executor.create_node("action_move_client")?;
    let logger = node.logger().clone();

    // 创建动作客户端（接口类型、动作名）
    let client = node.create_action_client::<MoveCircle>("move_circle")?;

    // 创建一个动作目标的消息
    let goal_msg = MoveCircle_Goal {
        // 设置动作目标为使能，希望机器人开始运动
        enable: true,
    };

    let logger_task = logger.clone();
    let promise = executor.commands().run(async move {
        log_info!(&logger_task, "发送动作目标...");

        // 使用 client.request_goal(goal_msg).await 发送目标，服务端拒绝时，返回None
        let Some(goal_client) = client.request_goal(goal_msg).await else {
            log_info!(&logger_task, "Goal rejected :(");
            return;
        };

        log_info!(&logger_task, "Goal accepted :)");
        let mut events = goal_client.stream();

        // 使用 GoalClient::stream() 接收反馈、状态和最终结果
        while let Some(event) = events.next().await {
            match event {
                GoalEvent::Feedback(feedback) => {
                    log_info!(&logger_task, "Received feedback: {}", feedback.state);
                }
                GoalEvent::Result((status, result)) => {
                    log_info!(
                        &logger_task,
                        "Result: finish={}, status={:?}",
                        result.finish,
                        status
                    );
                    break;
                }
                GoalEvent::Status(_) => {}
            }
        }
    });

    // 循环等待ROS2退出
    executor
        .spin(SpinOptions::default().until_promise_resolved(promise))
        .first_error()?;
    Ok(())
}
