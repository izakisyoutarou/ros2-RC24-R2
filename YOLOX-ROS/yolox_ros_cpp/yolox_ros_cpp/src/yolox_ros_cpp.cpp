#include "yolox_ros_cpp/yolox_ros_cpp.hpp"
//
namespace yolox_ros_cpp
{
    YoloXNode::YoloXNode(const rclcpp::NodeOptions &options)
        : Node("yolox_ros_cpp", options),
        str_range_point1(get_parameter("ditaction.str_range_point1").as_double_array()),
        str_range_point2(get_parameter("ditaction.str_range_point2").as_double_array()),
        str_range_x_C3orC5(get_parameter("ditaction.str_range_x_C3orC5").as_double_array()),
        str_ball_range_y(get_parameter("ditaction.str_ball_range_y").as_double()),
        siro_ball_range_y(get_parameter("ditaction.siro_ball_range_y").as_double_array()),
        siro_ball_range_x(get_parameter("ditaction.siro_ball_range_x").as_double_array())
    {
        using namespace std::chrono_literals; // NOLINT
        this->init_timer_ = this->create_wall_timer(
            0s, std::bind(&YoloXNode::onInit, this));
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

        this->sub_image_ = image_transport::create_subscription(
            this, this->params_.src_image_topic_name,
            std::bind(&YoloXNode::colorImageCallback, this, std::placeholders::_1),
            "raw");
        this->pub_bboxes_ = this->create_publisher<bboxes_ex_msgs::msg::BoundingBoxes>(
            this->params_.publish_boundingbox_topic_name,
            10);
        this->pub_image_ = image_transport::create_publisher(this, this->params_.publish_image_topic_name);

        _sub_viz_c1 = this->create_subscription<ditaction_interface_msg::msg::VisualizerC1>(
            "visualizer_c1",
            10,
            std::bind(&YoloXNode::callback_viz_c1, this, std::placeholders::_1)
        );

        _sub_viz_realsense = this->create_subscription<ditaction_interface_msg::msg::VisualizerRealsense>(
            "visualizer_realsense",
            10,
            std::bind(&YoloXNode::callback_viz_realsense, this, std::placeholders::_1)
        );
    }

    void YoloXNode::callback_viz_c1(const ditaction_interface_msg::msg::VisualizerC1::SharedPtr msg){
        str_from_upslope = msg->str_from_upslope;
        siro_form_upslope = msg->siro_form_upslope;
    }

    void YoloXNode::callback_viz_realsense(const ditaction_interface_msg::msg::VisualizerRealsense::SharedPtr msg){
        str_from_c3_c5 = msg->str_from_c3_c5;
        str_front_ball = msg->str_front_ball;
    }

    void YoloXNode::colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &ptr)
    {
        auto img = cv_bridge::toCvCopy(ptr, "bgr8");
        cv::Mat frame = img->image;

        auto now = std::chrono::system_clock::now();
        auto objects = this->yolox_->inference(frame);

        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - now);
        // RCLCPP_INFO(this->get_logger(), "Inference: %f FPS", 1000.0f / elapsed.count()); //FPSを確認できる

        std::vector<bool> viz_flag = {str_from_upslope, siro_form_upslope, str_from_c3_c5, str_front_ball};

        yolox_cpp::utils::draw_objects(
            frame, objects, this->class_names_, 
            str_range_point1, str_range_point2, str_range_x_C3orC5, siro_ball_range_y, siro_ball_range_x, str_ball_range_y,
            viz_flag);
        if (this->params_.imshow_isshow)
        {
            cv::imshow("yolox", frame);
            auto key = cv::waitKey(1);
            if (key == 27)
            {
                rclcpp::shutdown();
            }
        }

        auto boxes = objects_to_bboxes(frame, objects, img->header);

        for (auto box : boxes.bounding_boxes) {//しきい値のための新規追加
            if (box.probability > 0.9) {
                this->pub_bboxes_->publish(boxes);
                break;  // メッセージ全体を送信するため、不要ならばコメントアウトする
            }
        }

        sensor_msgs::msg::Image::SharedPtr pub_img;
        pub_img = cv_bridge::CvImage(img->header, "bgr8", frame).toImageMsg();
        this->pub_image_.publish(pub_img);
    }
    bboxes_ex_msgs::msg::BoundingBoxes YoloXNode::objects_to_bboxes(cv::Mat frame, std::vector<yolox_cpp::Object> objects, std_msgs::msg::Header header)
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
            boxes.bounding_boxes.emplace_back(box);
        }
        return boxes;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(yolox_ros_cpp::YoloXNode)
