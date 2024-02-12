#ifndef _YOLOX_CPP_UTILS_HPP
#define _YOLOX_CPP_UTILS_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "core.hpp"
#include "coco_names.hpp"

namespace yolox_cpp{
    namespace utils{

        static std::vector<std::string> read_class_labels_file(file_name_t file_name)
        {
            std::vector<std::string> class_names;
            std::ifstream ifs(file_name);
            std::string buff;
            if(ifs.fail()){
                return class_names;
            }
            while (getline(ifs, buff)) {
                if (buff == "")
                    continue;
                class_names.push_back(buff);
            }
            return class_names;
        }

        static void draw_objects(cv::Mat bgr, const std::vector<Object>& objects, const std::vector<std::string>& class_names=COCO_CLASSES,
            const std::vector<double>& str_range_point1 = std::vector<double>(), 
            const std::vector<double>& str_range_point2 = std::vector<double>(), 
            const std::vector<double>& str_range_x_C3orC5 = std::vector<double>(),
            const std::vector<double>& siro_ball_range_y = std::vector<double>(), 
            const std::vector<double>& siro_ball_range_x = std::vector<double>(), 
            const double str_ball_range_y = 0.0, 
            const std::vector<bool>& viz_flag = std::vector<bool>())
        {

            for (size_t i = 0; i < objects.size(); i++)
            {
                const Object& obj = objects[i];

                int color_index = obj.label % 80;
                cv::Scalar color = cv::Scalar(color_list[color_index][0], color_list[color_index][1], color_list[color_index][2]);
                float c_mean = cv::mean(color)[0];
                cv::Scalar txt_color;
                if (c_mean > 0.5){
                    txt_color = cv::Scalar(0, 0, 0);
                }else{
                    txt_color = cv::Scalar(255, 255, 255);
                }

                cv::rectangle(bgr, obj.rect, color * 255, 2);

                char text[256];
                sprintf(text, "%s %.1f%%", class_names[obj.label].c_str(), obj.prob * 100);

                int baseLine = 0;
                cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

                cv::Scalar txt_bk_color = color * 0.7 * 255;

                int x = obj.rect.x;
                int y = obj.rect.y + 1;
                if (y > bgr.rows)
                    y = bgr.rows;

                cv::rectangle(bgr, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                              txt_bk_color, -1);

                cv::putText(bgr, text, cv::Point(x, y + label_size.height),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, txt_color, 1);

                cv::Scalar line_color(0, 255, 0);  // BGR 色空間での色
                int x_coordinate = 680;//以下、新規追加。画面上に線を表示
                if(viz_flag[0]){
                    cv::line(bgr, cv::Point(x_coordinate, 0), cv::Point(x_coordinate, bgr.rows), line_color, 2);
                }
                else if(viz_flag[1]){
                    for (int i = 0; i < 6; i++){
                        cv::line(bgr, cv::Point(siro_ball_range_x[i], 0), cv::Point(siro_ball_range_x[i], bgr.rows), line_color, 2);
                    }
                    for (int i = 0; i < 3; i++){
                        cv::line(bgr, cv::Point(0, siro_ball_range_y[i]), cv::Point(bgr.cols, siro_ball_range_y[i]), line_color, 2);
                    }
                }
                else if(viz_flag[2]){
                    for (int i = 0; i < 3; i++){
                        cv::line(bgr, cv::Point(str_range_x_C3orC5[i], 0), cv::Point(str_range_x_C3orC5[i], bgr.rows), line_color, 2);
                    }
                }
                else if(viz_flag[3]){
                    cv::line(bgr, cv::Point(0, str_ball_range_y), cv::Point(bgr.cols, str_ball_range_y), line_color, 2);
                }
                
            }
        }
    }
}
#endif