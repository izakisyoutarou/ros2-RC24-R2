#include "yolox_ros_cpp/yolox_ros_cpp.hpp"
//
namespace yolox_ros_cpp
{
    YoloXNode::YoloXNode(const rclcpp::NodeOptions &options)
        : Node("yolox_ros_cpp", options)
    {
        using namespace std::chrono_literals; // NOLINT
        this->init_timer_ = this->create_wall_timer(
            8ms, std::bind(&YoloXNode::onInit, this));
    }

    void YoloXNode::onInit()
    {
        this->init_timer_->cancel();
        this->param_listener_ = std::make_shared<yolox_parameters::ParamListener>(
            this->get_node_parameters_interface());

        this->params_ = this->param_listener_->get_params();

        if (this->params_.imshow_isshow)
        {
            cv::namedWindow("yolox", cv::WINDOW_AUTOSIZE);
        }

        if (this->params_.class_labels_path != "")
        {
            RCLCPP_INFO(this->get_logger(), "read class labels from '%s'", this->params_.class_labels_path.c_str());
            this->class_names_ = yolox_cpp::utils::read_class_labels_file(this->params_.class_labels_path);
            for (const auto& class_name : class_names_) { //新規追加
                RCLCPP_INFO(rclcpp::get_logger("your_logger_name"), "Class Name: %s", class_name.c_str());
            }
        }
        else
        {
            this->class_names_ = yolox_cpp::COCO_CLASSES;
        }

        if (this->params_.model_type == "tensorrt")
        {
#ifdef ENABLE_TENSORRT
            RCLCPP_INFO(this->get_logger(), "Model Type is TensorRT");
            this->yolox_ = std::make_unique<yolox_cpp::YoloXTensorRT>(
                this->params_.model_path, this->params_.tensorrt_device,
                this->params_.nms, this->params_.conf, this->params_.model_version,
                this->params_.num_classes, this->params_.p6);
#else
            RCLCPP_ERROR(this->get_logger(), "yolox_cpp is not built with TensorRT");
            rclcpp::shutdown();
#endif
        }
        else if (this->params_.model_type == "openvino")
        {
#ifdef ENABLE_OPENVINO
            RCLCPP_INFO(this->get_logger(), "Model Type is OpenVINO");
            this->yolox_ = std::make_unique<yolox_cpp::YoloXOpenVINO>(
                this->params_.model_path, this->params_.openvino_device,
                this->params_.nms, this->params_.conf, this->params_.model_version,
                this->params_.num_classes, this->params_.p6);
#else
            RCLCPP_ERROR(this->get_logger(), "yolox_cpp is not built with OpenVINO");
            rclcpp::shutdown();
#endif
        }
        else if (this->params_.model_type == "onnxruntime")
        {
#ifdef ENABLE_ONNXRUNTIME
            RCLCPP_INFO(this->get_logger(), "Model Type is ONNXRuntime");
            this->yolox_ = std::make_unique<yolox_cpp::YoloXONNXRuntime>(
                this->params_.model_path,
                this->params_.onnxruntime_intra_op_num_threads,
                this->params_.onnxruntime_inter_op_num_threads,
                this->params_.onnxruntime_use_cuda, this->params_.onnxruntime_device_id,
                this->params_.onnxruntime_use_parallel,
                this->params_.nms, this->params_.conf, this->params_.model_version,
                this->params_.num_classes, this->params_.p6);
#else
            RCLCPP_ERROR(this->get_logger(), "yolox_cpp is not built with ONNXRuntime");
            rclcpp::shutdown();
#endif
        }
        else if (this->params_.model_type == "tflite")
        {
#ifdef ENABLE_TFLITE
            RCLCPP_INFO(this->get_logger(), "Model Type is tflite");
            this->yolox_ = std::make_unique<yolox_cpp::YoloXTflite>(
                this->params_.model_path, this->params_.tflite_num_threads,
                this->params_.nms, this->params_.conf, this->params_.model_version,
                this->params_.num_classes, this->params_.p6, this->params_.is_nchw);
#else
            RCLCPP_ERROR(this->get_logger(), "yolox_cpp is not built with tflite");
            rclcpp::shutdown();
#endif
        }
        RCLCPP_INFO(this->get_logger(), "model loaded");
        
        if(this->params_.depth_flag){
            _sub_rgbd = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
                "/camera/d455/rgbd",
                10,
                std::bind(&YoloXNode::RgbdCallback, this, std::placeholders::_1)
            );
        }
        else{
            this->sub_image_ = image_transport::create_subscription(
                this, this->params_.src_image_topic_name,
                std::bind(&YoloXNode::colorImageCallback, this, std::placeholders::_1),
                "raw");
        }
        this->pub_bboxes_ = this->create_publisher<bboxes_ex_msgs::msg::BoundingBoxes>(
            this->params_.publish_boundingbox_topic_name,
            10);
        this->pub_image_ = image_transport::create_publisher(this, this->params_.publish_image_topic_name);

        _sub_threshold = this->create_subscription<detection_interface_msg::msg::Threshold>(
            "threshold",
            10,
            std::bind(&YoloXNode::ThresholdCallback, this, std::placeholders::_1)
        );

        // _sub_rgbd = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
        //     "/camera/d455/rgbd",
        //     10,
        //     std::bind(&YoloXNode::RgbdCallback, this, std::placeholders::_1)
        // );
    }

    void YoloXNode::colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &ptr)
    {
        auto img = cv_bridge::toCvCopy(ptr, "bgr8");
        cv::Mat frame = img->image;
        cv::Mat frame_prev;

        cv::Mat depth_image;

        auto now = std::chrono::system_clock::now();
        auto objects = this->yolox_->inference(frame);

        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - now);
        // RCLCPP_INFO(this->get_logger(), "Inference: %f FPS", 1000.0f / elapsed.count()); //FPSを確認できる

        yolox_cpp::utils::draw_objects(xmin, ymin, xmax, frame, objects, this->class_names_);
        if (this->params_.imshow_isshow)
        {
            cv::Size target_size(1280, 720); // Set your desired size here
            cv::resize(frame, frame_prev, target_size);
            cv::imshow("yolox", frame_prev);
            auto key = cv::waitKey(1);
            if (key == 27)
            {
                rclcpp::shutdown();
            }
        }

        auto boxes = objects_to_bboxes(frame, depth_image, objects, img->header);

        this->pub_bboxes_->publish(boxes);
        
        sensor_msgs::msg::Image::SharedPtr pub_img;
        pub_img = cv_bridge::CvImage(img->header, "bgr8", frame).toImageMsg();
        this->pub_image_.publish(pub_img);
    }

    void YoloXNode::RgbdCallback(const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr &ptr){
        auto rgb_img = cv_bridge::toCvCopy(ptr->rgb, "bgr8");
        cv::Mat frame = rgb_img->image;
        cv::Mat frame_prev;

        auto depth_img = cv_bridge::toCvCopy(ptr->depth, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat depth_frame = depth_img->image;

        auto now = std::chrono::system_clock::now();
        auto objects = this->yolox_->inference(frame);

        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - now);
        //RCLCPP_INFO(this->get_logger(), "Inference: %f FPS", 1000.0f / elapsed.count()); //FPSを確認できる

        yolox_cpp::utils::draw_objects(xmin, ymin, xmax, frame, objects, this->class_names_);
        if (this->params_.imshow_isshow)
        {
            cv::Size target_size(1280, 720); // Set your desired size here
            cv::resize(frame, frame_prev, target_size);
            cv::imshow("yolox", frame_prev);
            auto key = cv::waitKey(1);
            if (key == 27)
            {
                rclcpp::shutdown();
            }
        }

        auto boxes = objects_to_bboxes(frame, depth_frame, objects, rgb_img->header);

        this->pub_bboxes_->publish(boxes);
        
        sensor_msgs::msg::Image::SharedPtr pub_img;
        pub_img = cv_bridge::CvImage(rgb_img->header, "bgr8", frame).toImageMsg();
        this->pub_image_.publish(pub_img);
    }

    bboxes_ex_msgs::msg::BoundingBoxes YoloXNode::objects_to_bboxes(cv::Mat frame, cv::Mat depth_image, std::vector<yolox_cpp::Object> objects, std_msgs::msg::Header header)
    {
        bboxes_ex_msgs::msg::BoundingBoxes boxes;
        boxes.header = header;
        for (auto obj : objects)
        {
            bboxes_ex_msgs::msg::BoundingBox box;
            box.probability = obj.prob;
            box.class_id = yolox_cpp::COCO_CLASSES[obj.label];
            box.xmin = obj.rect.x;
            box.ymin = obj.rect.y;
            box.xmax = (obj.rect.x + obj.rect.width);
            box.ymax = (obj.rect.y + obj.rect.height);
            box.img_width = frame.cols;
            box.img_height = frame.rows;

            if(this->params_.depth_flag){
                int center_value = 0;
                int center_x_value = static_cast<int>((box.xmax + box.xmin) / 2);
                int center_y_value = static_cast<int>((box.ymax + box.ymin) / 2);

                // std::cout << "center_x " << center_x_value << "center_y" << center_y_value << std::endl;

                // box.center_dist = depth_image.at<uint16_t>(center_y_value, center_x_value);

                if(center_x_value > 0 && center_x_value < 1200 && center_y_value > 0 && center_y_value < 700){
                    std::vector<uint16_t> center_dist = {   depth_image.at<uint16_t>(center_y_value, center_x_value), depth_image.at<uint16_t>(center_y_value-1, center_x_value), 
                                                            depth_image.at<uint16_t>(center_y_value-1, center_x_value+1), depth_image.at<uint16_t>(center_y_value, center_x_value+1),
                                                            depth_image.at<uint16_t>(center_y_value+1, center_x_value+1), depth_image.at<uint16_t>(center_y_value+1, center_x_value),
                                                            depth_image.at<uint16_t>(center_y_value+1, center_x_value-1), depth_image.at<uint16_t>(center_y_value, center_x_value-1),
                                                            depth_image.at<uint16_t>(center_y_value-1, center_x_value-1)};

                    std::sort(center_dist.begin(), center_dist.end());

                    center_value = center_dist[center_dist.size() / 2];

                    box.center_dist = center_value;

                    // center_dist.erase(std::remove(center_dist.begin(), center_dist.end(), 0), center_dist.end());

                    // // ゼロを削除した後、配列が空でない場合は平均を計算
                    // if (!center_dist.empty()) {
                    //     uint16_t sum = std::accumulate(center_dist.begin(), center_dist.end(), static_cast<uint16_t>(0));
                    //     uint16_t average = sum / center_dist.size();
                    //     // std::cout << "平均: " << average << std::endl;
                    //     box.center_dist = average;
                    // }
                }
            }

            boxes.bounding_boxes.emplace_back(box);
        }
        return boxes;
    }

    void YoloXNode::ThresholdCallback(const detection_interface_msg::msg::Threshold::ConstSharedPtr &ptr){
        xmax = ptr->xmax;
        xmin = ptr->xmin;
        ymin = ptr->ymin;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(yolox_ros_cpp::YoloXNode)
