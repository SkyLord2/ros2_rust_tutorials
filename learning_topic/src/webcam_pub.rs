use std::time::Duration;
use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, PublisherOptions, QoSProfile, QoSHistoryPolicy, log_info, log_error};
use sensor_msgs::msg::Image as ImageMsg;
use opencv::{
    videoio::{self, VideoCapture},
    prelude::*,
};
/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("webcam_pub")?;
    let logger = node.logger().clone();
    log_info!(&logger, "ROS2节点示例：发布节点");

    // 通过OpenCV打开摄像头
    let mut cam = VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera
    if !cam.is_opened()? {
        log_error!(&logger, "无法打开摄像头");
        return Err(anyhow::anyhow!("无法打开摄像头"));
    }

    let mut frame = Mat::default();
    
    let mut publisher_options = PublisherOptions::from("image_raw");
    let mut qos = QoSProfile::default();
    qos.history = QoSHistoryPolicy::KeepLast { depth: 20 };
    publisher_options.qos = qos;

    let publisher = node.create_publisher::<ImageMsg>(publisher_options)?;
    let timer_options = rclrs::TimerOptions::new(Duration::from_millis(100));
    let _timer = node.create_timer_repeating(timer_options, move || {
        // 从摄像头读取一帧
        if let Err(e) = cam.read(&mut frame) {
            log_error!(&logger, "读取摄像头失败: {:?}", e);
            return;
        }
        if frame.empty() {
            log_error!(&logger, "无法读取摄像头帧");
            return;
        }

        let mut img_msg = ImageMsg::default();

        if let Ok(size) = frame.size() {
            img_msg.header.frame_id = "webcam".to_string();
            img_msg.height = size.height as u32;
            img_msg.width = size.width as u32;
            img_msg.encoding = "bgr8".to_string(); // OpenCV默认是BGR格式
            img_msg.is_bigendian = 0;

            if let Ok(elem_size) = frame.elem_size() {
                img_msg.step = (size.width as u32) * (elem_size as u32);
            } else {
                // 如果获取失败，对于 bgr8 默认降级使用 width * 3
                img_msg.step = (size.width * 3) as u32; 
            }

            if let Ok(data_bytes) = frame.data_bytes() {
                img_msg.data = data_bytes.to_vec();
                if let Err(err) = publisher.publish(img_msg) {
                    log_error!(&logger, "发布摄像头图像失败: {:?}", err);
                }
            } else {
                log_error!(&logger, "无法获取摄像头帧数据");
                return;
            }

        }
    })?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
