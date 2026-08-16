use std::sync::atomic::{AtomicI32, Ordering};
use std::time::Duration;
use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, log_info, log_error};
use sensor_msgs::msg::Image as ImageMsg;
use learning_interface::msg::ObjectPosition as ObjectPositionMsg;
use opencv::{
    core::{self, Point, Scalar, Vector},
    highgui, imgproc,
    prelude::*,
};

static POSITION_X: AtomicI32 = AtomicI32::new(0);
static POSITION_Y: AtomicI32 = AtomicI32::new(0);

fn object_detect(image: &mut Mat) -> Result<()> {
    let lower_red = Scalar::new(0.0, 90.0, 128.0, 0.0);
    let upper_red = Scalar::new(180.0, 255.0, 255.0, 0.0);
    // 图像从BGR颜色模型转换为HSV模型
    let mut hsv_img = Mat::default();
    imgproc::cvt_color(image, &mut hsv_img, imgproc::COLOR_BGR2HSV, 0)?;
    // 图像二值化
    let mut mask = Mat::default();
    core::in_range(&hsv_img, &lower_red, &upper_red, &mut mask)?;
    // 图像中轮廓检测
    let mut contours = Vector::<Vector<Point>>::new();
    imgproc::find_contours(
        &mask,
        &mut contours,
        imgproc::RETR_LIST,
        imgproc::CHAIN_APPROX_NONE,
        Point::new(0, 0),
    )?;
    // 去除一些轮廓面积太小的噪声
    for (i, cnt) in contours.iter().enumerate() {
        // cnt.len() 对应 Python 中 cnt.shape[0]（轮廓点数量）
        if cnt.len() < 150 {
            continue;
        }

        // 得到苹果所在轮廓的左上角xy像素坐标及轮廓范围的宽和高
        let rect = imgproc::bounding_rect(&cnt)?;
        let center_x = rect.x + rect.width / 2;
        let center_y = rect.y + rect.height / 2;

        POSITION_X.store(center_x, Ordering::Relaxed);
        POSITION_Y.store(center_y, Ordering::Relaxed);

        // 将苹果的轮廓勾勒出来
        imgproc::draw_contours(
            image,
            &contours,
            i as i32,                           // 指定绘制第 i 个轮廓
            Scalar::new(0.0, 255.0, 0.0, 0.0), // BGR 颜色：绿色
            2,                                 // 线宽为 2
            imgproc::LINE_8,
            &core::Mat::default(),
            0,
            Point::new(0, 0),
        )?;

        // 将苹果的图像中心点画出来
        imgproc::circle(
            image,
            Point::new(center_x, center_y),
            5,                                 // 半径为 5
            Scalar::new(0.0, 255.0, 0.0, 0.0), // 绿色
            -1,                                // -1 表示实心填充
            imgproc::LINE_8,
            0,
        )?;
    }
    // 使用OpenCV显示处理后的图像效果
    highgui::imshow("object", image)?;
    highgui::wait_key(10)?;
    Ok(())
}

/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("object_pub")?;
    let logger = node.logger().clone();
    log_info!(&logger, "ROS2节点示例：发布节点");

    let timer_logger = logger.clone();

    let _publisher = node.create_publisher::<ObjectPositionMsg>("object_position")?;
    let timer_options = rclrs::TimerOptions::new(Duration::from_millis(100));
    let _timer = node.create_timer_repeating(timer_options, move || {
        let mut msg = ObjectPositionMsg::default();
        msg.x = POSITION_X.load(Ordering::Relaxed);
        msg.y = POSITION_Y.load(Ordering::Relaxed);
        if let Err(err) = _publisher.publish(msg) {
            log_error!(&timer_logger, "发布物体位置失败: {:?}", err);
        }
    });

    let _subscription = node.create_subscription::<ImageMsg, _>(
        "image_raw",
        move |msg: ImageMsg| {
            log_info!(&logger, "收到图片消息: {}x{}", msg.height, msg.width);
            // Process the image data here
            if let Ok(frame_1d) = Mat::from_slice(&msg.data) {
                // bgr8 对应 3 个通道，高度对应 msg.height
                if let Ok(reshaped_frame) = frame_1d.reshape(3, msg.height as i32) {
                    let mut frame = Mat::default();
                    if let Err(e) = reshaped_frame.copy_to(&mut frame) {
                        log_error!(&logger, "图像矩阵拷贝失败: {:?}", e);
                        return;
                    }
                    if let Err(e) = object_detect(&mut frame) {
                        log_error!(&logger, "图像处理失败: {:?}", e);
                    }
                } else {
                    log_error!(&logger, "无法将一维数据重塑为二维图像矩阵");
                }
            } else {
                 log_error!(&logger, "无法从原始字节流生成矩阵");
            }
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
