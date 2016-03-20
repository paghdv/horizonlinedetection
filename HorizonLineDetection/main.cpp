#include "../src/HorizonLineDetector.h"
#include <iostream>

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

    if (is_run)
    {
        std::string training_file="../Training/saved_model.svm";
        std::string file_path_mask="";
        if (argc>5)
            file_path_mask=argv[5];
        if (argc>6)
            training_file=argv[6];
        //Test
        cv::Mat mask;
        if (!file_path_mask.empty())
            mask=cv::imread(file_path_mask);
        if (!mask.empty())
            std::cout<<"Using mask"<<std::endl;
        hld.init_detector(training_file);
        hld.detect_video(file_path_in,file_path_out,mask);
    }
    else
    {
        hld.train(file_path_in);
        hld.save_model(file_path_out);
    }

}
