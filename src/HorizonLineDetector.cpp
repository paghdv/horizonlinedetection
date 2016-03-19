#include "HorizonLineDetector.h"
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>

HorizonLineDetector::HorizonLineDetector()
{
    params.svm_type    = CvSVM::C_SVC;
    params.kernel_type = CvSVM::LINEAR;
    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
    first_node = std::make_shared<Node>();
    first_node->cost=0;
    first_node->prev=nullptr;
    last_node = std::make_shared<Node>();
    last_node->cost=INFINITY;
    last_node->prev=nullptr;
}

void HorizonLineDetector::find_edge_list(const cv::Mat& mask)
{
    current_keypoints.clear();
    const bool use_mask=!mask.empty();
    assert(current_edges.cols > 0 && current_edges.rows > 0 &&
           current_edges.channels() == 1 && current_edges.depth() == CV_8U
           && (mask.depth() == CV_8U || mask.empty()));
    const int M = current_edges.rows;
    const int N = current_edges.cols;
    const float MN=M*N;
    const char* bin_ptr = current_edges.ptr<char>();
    char* mask_ptr;
    if (use_mask)
        mask_ptr = (char*)mask.ptr<char>();

    for (int m = 0; m < MN; m++)
    {
        if (bin_ptr[m] != 0 && (!use_mask || mask_ptr[m]!=0))
                current_keypoints.push_back(cv::KeyPoint(m%N,m/N,10.0f));

    }
}

bool HorizonLineDetector::dp(std::shared_ptr<Node> n)
{
    //Check if I've been here already and took a cheaper path (already)
    const int curr_visited=visited(n->y,n->x);
    if ( (curr_visited>-1 && curr_visited <= n->cost) || n->cost>=last_node->cost ) return false;
    visited(n->y,n->x) = n->cost;
    //Check if we reached the last column(done!)
    if (n->x==current_edges.cols-1)
    {
        //Give preference to the points ending on the upper parts of the images
        n->cost+=n->y;
        //Save the info in the last node if it's the cheapest path
        if (last_node->cost > n->cost)
        {
            last_node->cost=n->cost;
            last_node->prev=n;
        }
        return true;
    }
    else
    {
        bool good_neighbors=false;
        //Check for neighboring pixels to see if they are edges, launch dp with all the ones that are
        for (int i=1;i<2;i++)
        {
            for (int j=-1;j<2;j++)
            {
                if (i==0 && j==0) continue;
                if (n->x+i >= current_edges.cols || n->x+i < 0 ||
                    n->y+j >= current_edges.rows || n->y+j < 0) continue;
                if (current_edges.at<char>(n->y+j,n->x+i)!=0)
                {
                    good_neighbors=true;
                    auto n1=std::make_shared<Node>(n,n->x+i,n->y+j);
                    n1->cost= (max_cost-current_edges.at<char>(n1->y,n1->x)) + n->cost;
                    ntree.insert(std::pair<int,std::shared_ptr<Node> >(n1->cost/n1->x,n1));
                    //nlist.push_back(n1);
                }
            }
        }
        //If there are no edge neighbors go search for gold!
        if (!good_neighbors)
        {
            //If I've been lost for too long stop searching
            if (n->lost>=max_lost_steps)
                return false;
            //Otherwise continue seraching
            for (int i=1;i<2;i++)
            {
                for (int j=-1;j<2;j++)
                {
                    if (i==0 && j==0) continue;
                    if (n->x+i >= current_edges.cols || n->x+i < 0 ||
                        n->y+j >= current_edges.rows || n->y+j < 0) continue;

                    auto n1=std::make_shared<Node>(n,n->x+i,n->y+j);
                    n1->lost=1+n->lost;
                    n1->cost= lost_step_cost + n->cost;
                    ntree.insert(std::pair<int,std::shared_ptr<Node> >(n1->cost/n1->x,n1));
                    //nlist.push_back(n1);
                }
            }
        }
    }
    return false;
}

void HorizonLineDetector::add_node_to_horizon(std::shared_ptr<Node> n)
{
    cv::Point2i p1;
    p1.x=n->x;
    p1.y=n->y;
    horizon.push_back(p1);
    if (n->prev!=nullptr)
        add_node_to_horizon(n->prev);
}

bool HorizonLineDetector::compute_cheapest_path(const cv::Mat &mask)
{
    horizon.clear();
    visited=cv::Mat_<int>::zeros(current_edges.rows,current_edges.cols);
    visited.setTo(-1,mask);

    //Take all the edges at the left of the image and initialize paths
    //First check paths that start with edges identified
    for (size_t i=max_lost_steps;i<current_edges.rows-max_lost_steps;i++)
    {
        int cost=max_cost-current_edges.at<char>(i,0);
        auto n=std::make_shared<Node>(first_node,0,i);//(first_node,0,i);
        int top=0;
        if (cost==max_cost)
        {
            n->lost=1;
            top=i;
            cost=lost_step_cost;
        }
        n->cost=cost;
        ntree.insert(std::pair<int,std::shared_ptr<Node>>(n->cost+top,n));

        //nlist.push_back(n);
    }
    std::map<int,std::shared_ptr<Node>>::iterator curr_node;//Iterator

    curr_node=ntree.begin();
    bool found=false;
    //Start expanding shortest paths first
    while(curr_node!=ntree.end())
    {
        found=dp(curr_node->second);
        //Move to next leaft
        ntree.erase(curr_node);
        curr_node=ntree.begin();
    }

    if (last_node->prev!=nullptr)
        add_node_to_horizon(last_node->prev);
    reset_dp();
}

void HorizonLineDetector::compute_edges(const cv::Mat &mask,const int lowThreshold)
{
    int ratio = 3;
    int kernel_size = 3;
    cv::GaussianBlur(current_frame, current_edges, cv::Size(7,7), 1.5, 1.5);

    cv::Canny( current_edges, current_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    if (!mask.empty())
        current_edges=current_edges.mul(mask);

    cv::imwrite("edges.png",current_edges);
    find_edge_list();
}

void HorizonLineDetector::compute_descriptors()
{
    cv::Mat descriptorsMat_;

    extractor.compute(current_frame, current_keypoints, descriptorsMat_ );
    descriptorsMat_.convertTo(descriptorsMat,CV_32F);
}

void HorizonLineDetector::compute_dp_paths()
{

}
bool HorizonLineDetector::train(const std::string training_list_file)
{
    int n;
	//Load traing mat and label mat
    std::fstream inpt(training_list_file.c_str(),std::fstream::in);
    if (!inpt.is_open())
        return false;
    inpt>>n;
    cv::Mat mask,mask2;

    for (int i=0;i<n;i++)
    {
        cv::Mat temp_labels_pos,temp_labels_neg;
        std::string img_name, edge_name;
        inpt>>img_name;
        inpt>>edge_name;

        //Read images
        current_frame = cv::imread(img_name, CV_LOAD_IMAGE_GRAYSCALE);
        mask  = cv::imread(edge_name,CV_LOAD_IMAGE_GRAYSCALE);
        mask2=255-mask;
        //Compute edges
        compute_edges(mask);
        //current_edges=mask;
        //find_edge_list();
        compute_descriptors();
        //Copy the matrix of descriptors
        //trainingDataMat
        trainingDataMat.push_back(descriptorsMat);
        temp_labels_pos=cv::Mat::ones(descriptorsMat.rows,1,CV_32FC1);
        labelsMat.push_back(temp_labels_pos);
        //Set all as positive labels
        //labelsMat

        compute_edges(mask2,20);
        imwrite("mask2.jpg",mask2);
        //current_edges=255-mask;
        //find_edge_list();
        compute_descriptors();
        //Concatenate features matrix
        trainingDataMat.push_back(descriptorsMat);
        temp_labels_neg=-cv::Mat::ones(descriptorsMat.rows,1,CV_32FC1);
        labelsMat.push_back(temp_labels_neg);

    }

    //Train SVM
    svm.train_auto(trainingDataMat,labelsMat,cv::Mat(), cv::Mat(), params);

	return true;
}

bool HorizonLineDetector::init_detector(const std::string training_config_file)
{
    return load_model(training_config_file);
}

bool HorizonLineDetector::save_model(const std::string config_file)
{
    svm.save(config_file.c_str());
	return true;
}
bool HorizonLineDetector::load_model(const std::string config_file)
{
    svm.load(config_file.c_str());
    return true;
}
void HorizonLineDetector::detect_image(const cv::Mat &frame,const cv::Mat &mask)
{
    if (!mask.empty() && mask.channels()>1)
    {
        std::cout<<"ERROR: Input mask should be CV_8U"<<std::endl;
        return;
    }
    if (frame.channels()>1)
		cvtColor(frame, current_frame, CV_BGR2GRAY);
    else
        current_frame=frame;
    compute_edges(mask,10);
    compute_descriptors();
    valid_edges.resize(current_keypoints.size());
    int val;
    cv::threshold(current_edges,current_edges,1,1,CV_8U);
    for (size_t i=0;i<valid_edges.size();i++)
    {
        valid_edges[i]= svm.predict(descriptorsMat.row(i))==1;
        if (valid_edges[i])
            current_edges.at<char>(current_keypoints[i].pt.y,current_keypoints[i].pt.x)=2;
    }
    //cv::imwrite("edges2.png",current_edges);
    compute_cheapest_path(mask);
}
bool HorizonLineDetector::detect_video(const std::string video_file,const std::string video_file_out,const cv::Mat &mask_)
{
    cv::Mat mask;
    if (!mask_.empty())
    {
        if (mask_.channels()>1)
            cvtColor(mask_, mask, CV_BGR2GRAY);
        else
            mask_.copyTo(mask);

        if (mask.type()!=CV_8U)
            mask.convertTo(mask,CV_8U);
    }
    cv:: VideoCapture cap(video_file); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
        	return false;

    cv::Size S = cv::Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
                      (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT));
    //int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
    int ex = CV_FOURCC('X','V','I','D');
    //int ex = CV_FOURCC('X','2','6','4');
    //cv::VideoWriter wrt(video_file_out, ex, cap.get(CV_CAP_PROP_FPS), S,false);
    int i=0;
	for(;;)
	{
		cv::Mat frame;
		cap >> frame; // get a new frame from camera
        detect_image(frame,mask);
        draw_horizon();
        //wrt.write(current_draw);
        std::stringstream ss;
        ss<<i;
        std::cout<<i<<" "<<std::endl;
        save_draw_frame(video_file_out+ss.str()+".png");
        i++;
	}

    //wrt.release();
	return true;
}

void HorizonLineDetector::draw_horizon()
{
    const cv::Scalar s1(0,255,0);
    cvtColor(current_frame,current_draw, CV_GRAY2RGB);
    for (size_t i=0;i<horizon.size();i++)
    {
        if (horizon[i].x+1 < current_edges.cols && horizon[i].y+1 < current_edges.rows)
            current_draw.colRange(horizon[i].x,horizon[i].x+1)
                        .rowRange(horizon[i].y,horizon[i].y+1)=s1;
    }
}

void HorizonLineDetector::draw_edges()
{
    const cv::Scalar s1(0,255,0),s2(255,0,0);
    cvtColor(current_frame,current_draw, CV_GRAY2RGB);
    for (size_t i=0;i<current_keypoints.size();i++)
        if(valid_edges[i])
            current_draw.colRange(current_keypoints[i].pt.x,current_keypoints[i].pt.x+1)
                        .rowRange(current_keypoints[i].pt.y,current_keypoints[i].pt.y+1)=s1;
        else
            current_draw.colRange(current_keypoints[i].pt.x,current_keypoints[i].pt.x+1)
                        .rowRange(current_keypoints[i].pt.y,current_keypoints[i].pt.y+1)=s2;
}

void HorizonLineDetector::save_draw_frame(const std::string file_name)
{
    cv::imwrite(file_name,current_draw);
}

void HorizonLineDetector::reset_dp()
{
    delete_nodes();
    first_node->cost=0;
    first_node->prev=nullptr;
    last_node->cost=INFINITY;
}

void HorizonLineDetector::delete_nodes()
{
    /*
    for (size_t i=0;i<n->next.size();i++)
        delete_nodes(n->next[i]);

    n->next.clear();
    n->prev=nullptr;
    */
    /*
    for (std::list<std::shared_ptr<Node> >::iterator it=nlist.begin(); it != nlist.end(); ++it)
        it->get()->prev=nullptr;
    nlist.clear();
    */
    last_node->prev=nullptr;
    ntree.clear();
}
