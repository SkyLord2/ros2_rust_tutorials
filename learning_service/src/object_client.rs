use anyhow::Result;
use rclrs::{log_info, Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use std::time::Duration;
// 导入自定义的服务接口数据结构
use learning_interface::srv::{GetObjectPosition, GetObjectPosition_Request, GetObjectPosition_Response};

fn main() -> Result<()> {
    // 创建 ROS 2 上下文
    let context: Context = Context::default_from_env()?;
    // 创建一个基本的执行器
    let mut executor = context.create_basic_executor();
    // 创建一个节点
    let node = executor.create_node("object_client")?;
    let logger = node.logger();
    log_info!(logger, "Hello from learning_client package!");

    // 创建一个客户端，指定服务类型和服务名称
    let client = node.create_client::<GetObjectPosition>("get_object_position")?;

    // 构造请求对象
    let request = GetObjectPosition_Request { get: true };

    while !client.service_is_ready()? {
        log_info!(logger, "service not available, waiting again...");
        std::thread::sleep(Duration::from_secs(1));
    }

    // 发送请求并获取响应 promise
    let mut response_promise = client.call::<_, GetObjectPosition_Response>(request)?;

    // 非阻塞地轮询响应，直到收到结果或发生错误
    loop {
        executor
            .spin(SpinOptions::spin_once().timeout(Duration::from_millis(100)))
            .first_error()?;

        // if let Some(response) = response_promise.try_recv()? {
        //     log_info!(logger, "Response received: x={}, y={}", response.x, response.y);
        //     break;
        // }
        match response_promise.try_recv() {
            Ok(Some(response)) => {
                log_info!(logger, "Response received: x={}, y={}", response.x, response.y);
                break;
            }
            Ok(None) => {
                // Response not yet available, continue spinning
            }
            Err(e) => {
                log_info!(logger, "Error receiving response: {:?}", e);
                break;
            }
        }
    }

    Ok(())
}
