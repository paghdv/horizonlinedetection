#include "../src/HorizonLineDetector.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[])
{
    //Usage: HorizonLineDetector mode file_path_in file_path_out
    // where mode: 0 (training), 1 (running)
    //       file_path_in will be a list file in the case of training and a video file path in the case of running
    //       file_path_out will config file output in the case of training and a video file path in the case of running
    HorizonLineDetector hld;

    const bool is_run=atoi(argv[1])==1;
    const std::string file_path_in=argv[2];
    const std::string file_path_out=argv[3];
    std::string file_path_mask="";
    if (argc>4)
        file_path_mask=argv[4];
    if (is_run)
    {
        //Test
        //cv::Mat test_frame=cv::imread(file_path_in);
        cv::Mat mask;
        if (!file_path_mask.empty())
            mask=cv::imread(file_path_mask);
        if (!mask.empty())
            std::cout<<"Using mask"<<std::endl;
        hld.init_detector("../Training/saved_model.svm");
        hld.detect_video(file_path_in,file_path_out,mask);
        /*
        hld.detect_image(test_frame);
        hld.draw_horizon();
        hld.save_draw_frame(file_path_out);
        hld.draw_edges();
        hld.save_draw_frame("candidates.png");
        */
    }
    else
    {
        hld.train(file_path_in);
        hld.save_model("../Training/saved_model.svm");
    }

}
