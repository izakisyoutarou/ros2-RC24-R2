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

        static void draw_objects(cv::Mat bgr, const std::vector<Object>& objects, const std::vector<std::string>& class_names=COCO_CLASSES)
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

                //ここで補助ラインの描写
                cv::Scalar line_color(0, 255, 0);  // BGR 色空間での色

                //ひし形近く
                // int x_coordinate0 = 230;
                // int x_coordinate1 = 390;
                // int x_coordinate2 = 560;
                // int x_coordinate3 = 710;
                // int x_coordinate4 = 860;
                // cv::line(bgr, cv::Point(x_coordinate0, 0), cv::Point(x_coordinate0, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(x_coordinate1, 0), cv::Point(x_coordinate1, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(x_coordinate2, 0), cv::Point(x_coordinate2, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(x_coordinate3, 0), cv::Point(x_coordinate3, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(x_coordinate4, 0), cv::Point(x_coordinate4, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(0, 230), cv::Point(bgr.cols, 230), line_color, 2);
                // cv::line(bgr, cv::Point(0, 450), cv::Point(bgr.cols, 450), line_color, 2);

                //ひし形前後
                // cv::line(bgr, cv::Point(0, 370), cv::Point(bgr.cols, 370), line_color, 2);
                // cv::line(bgr, cv::Point(0, 520), cv::Point(bgr.cols, 520), line_color, 2);
                // cv::line(bgr, cv::Point(350, 0), cv::Point(350, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(580, 0), cv::Point(580, bgr.rows), line_color, 2);

                //坂上
                // cv::line(bgr, cv::Point(1280, 540), cv::Point(0, 180), line_color, 2); //realsense
                // cv::line(bgr, cv::Point(0, 580), cv::Point(1920, 1120),  line_color, 2); //c1 坂上斜め赤
                // cv::line(bgr, cv::Point(0, 1170), cv::Point(1920, 630),  line_color, 2); //c1 坂上斜め青
                // cv::line(bgr, cv::Point(0, 1115), cv::Point(1920, 570),  line_color, 2); //c1 囲い
                // cv::line(bgr, cv::Point(0, 1310), cv::Point(1920, 700),  line_color, 2); //c1 囲い
                // cv::line(bgr, cv::Point(0, 650), cv::Point(1920, 1250),  line_color, 2); //c1 囲い
                // cv::line(bgr, cv::Point(0, 590), cv::Point(1920, 1020),  line_color, 2); //c1 囲い

                // サイロ
                // cv::line(bgr, cv::Point(280, 0), cv::Point(280, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(440, 0), cv::Point(440, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(600, 0), cv::Point(600, bgr.rows), line_color, 2);                
                // cv::line(bgr, cv::Point(750, 0), cv::Point(750, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(970, 0), cv::Point(970, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(1110, 0), cv::Point(1110, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(1340, 0), cv::Point(1340, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(1500, 0), cv::Point(1500, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(1670, 0), cv::Point(1670, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(1810, 0), cv::Point(1810, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(0, 620), cv::Point(bgr.cols, 620), line_color, 2);
                // cv::line(bgr, cv::Point(0, 710), cv::Point(bgr.cols, 710), line_color, 2);
                // cv::line(bgr, cv::Point(0, 800), cv::Point(bgr.cols, 800), line_color, 2);
                // cv::line(bgr, cv::Point(0, 900), cv::Point(bgr.cols, 900), line_color, 2);
            }
        }
    }
}
#endif