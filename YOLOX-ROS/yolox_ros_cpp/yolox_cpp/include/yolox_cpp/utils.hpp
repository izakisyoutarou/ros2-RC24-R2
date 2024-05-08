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

        static void draw_objects(const int xmin, const int ymin, const int xmax, cv::Mat bgr, const std::vector<Object>& objects, const std::vector<std::string>& class_names=COCO_CLASSES)
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

                int center_line = (xmax+xmin)/2;

                int c1_kakoi = 0;

                cv::Point back_point(500, 650);
                cv::circle(bgr, back_point, 2, line_color, -1); 

                // //c3、c4
                // cv::line(bgr, cv::Point(xmin, 0), cv::Point(xmin, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point((center_line+xmin)/2, 0), cv::Point((center_line+xmin)/2, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(center_line, 0), cv::Point(center_line, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point((center_line+xmax)/2, 0), cv::Point((center_line+xmax)/2, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(xmax, 0), cv::Point(xmax, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(0, ymin), cv::Point(bgr.cols, ymin), line_color, 2);
                // cv::line(bgr, cv::Point(0, ymin+160), cv::Point(bgr.cols, ymin+160), line_color, 2);
                // cv::line(bgr, cv::Point(0, ymin+320), cv::Point(bgr.cols, ymin+320), line_color, 2);//カメラの向きが上下逆

                //ST1~8
                // cv::line(bgr, cv::Point(550, 0), cv::Point(550, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(800, 0), cv::Point(800, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(0, 340), cv::Point(bgr.cols, 340), line_color, 2);
                // cv::line(bgr, cv::Point(0, 500), cv::Point(bgr.cols, 500), line_color, 2);

                //坂上
                cv::line(bgr, cv::Point(1280, 540), cv::Point(0, 180), line_color, 2); //realsense
                cv::line(bgr, cv::Point(0, 630 + c1_kakoi), cv::Point(1920, 1230 + c1_kakoi),  line_color, 2); //c1 坂上斜め赤
                // cv::line(bgr, cv::Point(0, 1170), cv::Point(1920, 630),  line_color, 2); //c1 坂上斜め青
                cv::line(bgr, cv::Point(0, 1180 + c1_kakoi), cv::Point(1920, 560 + c1_kakoi),  line_color, 2); //c1 囲い hidariue 
                cv::line(bgr, cv::Point(0, 1650 + c1_kakoi), cv::Point(1920, 590 + c1_kakoi),  line_color, 2); //c1 囲い migisita
                cv::line(bgr, cv::Point(0, 650 + c1_kakoi), cv::Point(1920, 1550 + c1_kakoi),  line_color, 2); //c1 囲い hidarisita
                cv::line(bgr, cv::Point(0, 620 + c1_kakoi), cv::Point(1920, 1100 + c1_kakoi),  line_color, 2); //c1 囲い migiue

                // サイロ
                // cv::line(bgr, cv::Point(350, 0), cv::Point(350, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(768, 0), cv::Point(768, bgr.rows), line_color, 2);
                // cv::line(bgr, cv::Point(1220, 0), cv::Point(1220, bgr.rows), line_color, 2);               
                // cv::line(bgr, cv::Point(1600, 0), cv::Point(1600, bgr.rows), line_color, 2);                
                // cv::line(bgr, cv::Point(0, 780), cv::Point(bgr.cols, 780), line_color, 2);
                // cv::line(bgr, cv::Point(0, 900), cv::Point(bgr.cols, 900), line_color, 2);
                // cv::line(bgr, cv::Point(0, 1050), cv::Point(bgr.cols, 1050), line_color, 2);
            }
        }
    }
}
#endif