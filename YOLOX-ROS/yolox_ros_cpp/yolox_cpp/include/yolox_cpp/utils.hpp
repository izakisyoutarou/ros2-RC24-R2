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

                int c1_kakoi = 0;

                //c3、c4
                // cv::line(bgr, cv::Point(390, 0), cv::Point(390, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(560, 0), cv::Point(560, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(710, 0), cv::Point(710, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(0, 250), cv::Point(bgr.cols, 250), line_color, 2);

                //ST1~8
                // cv::line(bgr, cv::Point(390, 0), cv::Point(390, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(560, 0), cv::Point(560, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(0, 350), cv::Point(bgr.cols, 350), line_color, 2);
                // cv::line(bgr, cv::Point(0, 500), cv::Point(bgr.cols, 500), line_color, 2);

                //ひし形前後
                // cv::line(bgr, cv::Point(0, 370), cv::Point(bgr.cols, 370), line_color, 2);
                // cv::line(bgr, cv::Point(0, 520), cv::Point(bgr.cols, 520), line_color, 2);
                // cv::line(bgr, cv::Point(350, 0), cv::Point(350, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(580, 0), cv::Point(580, bgr.rows), line_color, 2);

                //坂上
                // cv::line(bgr, cv::Point(1280, 540), cv::Point(0, 180), line_color, 2); //realsense
                cv::line(bgr, cv::Point(0, 580 + c1_kakoi), cv::Point(1920, 1120 + c1_kakoi),  line_color, 2); //c1 坂上斜め赤
                // cv::line(bgr, cv::Point(0, 1170), cv::Point(1920, 630),  line_color, 2); //c1 坂上斜め青
                cv::line(bgr, cv::Point(0, 1230 + c1_kakoi), cv::Point(1920, 460 + c1_kakoi),  line_color, 2); //c1 囲い hidariue 
                cv::line(bgr, cv::Point(0, 1550 + c1_kakoi), cv::Point(1920, 620 + c1_kakoi),  line_color, 2); //c1 囲い migisita
                cv::line(bgr, cv::Point(0, 650 + c1_kakoi), cv::Point(1920, 1450 + c1_kakoi),  line_color, 2); //c1 囲い hidarisita
                cv::line(bgr, cv::Point(0, 590 + c1_kakoi), cv::Point(1920, 1020 + c1_kakoi),  line_color, 2); //c1 囲い hidariue

                // サイロ
                // cv::line(bgr, cv::Point(520, 0), cv::Point(520, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(768, 0), cv::Point(768, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(1152, 0), cv::Point(1152, bgr.rows), line_color, 2);                
                // cv::line(bgr, cv::Point(1450, 0), cv::Point(1450, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(0, 700), cv::Point(bgr.cols, 700), line_color, 2);
                // cv::line(bgr, cv::Point(0, 780), cv::Point(bgr.cols, 780), line_color, 2);
                // cv::line(bgr, cv::Point(0, 880), cv::Point(bgr.cols, 880), line_color, 2);
            }
        }
    }
}
#endif