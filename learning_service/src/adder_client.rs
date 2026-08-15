use anyhow::Result;
use rclrs::{log_info, Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use std::env;
use std::time::Duration;
// 导入自定义的服务接口数据结构
use learning_interface::srv::{AddTwoInts, AddTwoInts_Request, AddTwoInts_Response};

fn main() -> Result<()> {
    // 创建 ROS 2 上下文
    let context: Context = Context::default_from_env()?;
    // 创建一个基本的执行器
    let mut executor = context.create_basic_executor();
    // 创建一个节点
    let node = executor.create_node("adder_client")?;
    let logger = node.logger();
    log_info!(logger, "Hello from learning_client package!");

    // 创建一个客户端，指定服务类型和服务名称
    let client = node.create_client::<AddTwoInts>("add_two_ints")?;

    // 从命令行参数获取两个整数，默认值为 0
    let args: Vec<String> = env::args().collect();
    let a: i64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(0);
    let b: i64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(0);

    // 构造请求对象
    let request = AddTwoInts_Request { a, b };

    while !client.service_is_ready()? {
        log_info!(logger, "service not available, waiting again...");
        std::thread::sleep(Duration::from_secs(1));
    }

    // 发送请求并获取响应 promise
    let mut response_promise = client.call::<_, AddTwoInts_Response>(request)?;

    // 非阻塞地轮询响应，直到收到结果或发生错误
    loop {
        executor
            .spin(SpinOptions::spin_once().timeout(Duration::from_millis(100)))
            .first_error()?;

        if let Some(response) = response_promise.try_recv()? {
            log_info!(logger, "Response received: sum={}", response.sum);
            break;
        }
    }

    Ok(())
}
