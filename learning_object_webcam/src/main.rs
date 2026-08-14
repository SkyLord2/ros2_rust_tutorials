use anyhow::Result;
use opencv::{
    core::{self, Point, Scalar, Vector},
    videoio::{self, VideoCapture},
    highgui, imgproc,
    prelude::*,
};
// RclrsErrorFilter, SpinOptions,
use rclrs::{Context, CreateBasicExecutor, log_info, log_error};

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

    Ok(())
}
/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("learning_object_webcam_node")?;
    log_info!(node.logger(), "ROS2节点示例：检测图片中的苹果");

    let mut cam = VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera
    if !cam.is_opened()? {
        log_error!(node.logger(), "无法打开摄像头");
        return Err(anyhow::anyhow!("无法打开摄像头"));
    }
    
    let mut frame = Mat::default();

    while context.ok() {
        cam.read(&mut frame)?;
        if frame.empty() {
            log_error!(node.logger(), "无法读取摄像头帧");
            continue;
        }

        object_detect(&mut frame)?;

        let key = highgui::wait_key(10)?;
        if key == 113 {
            break;
        }
    }
    highgui::destroy_all_windows()?;
    // executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
