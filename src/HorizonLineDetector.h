#include "opencv2/core.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgcodecs.hpp"

#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <memory>
#include <map>
#include <list>
#ifndef  HORIZONLINEDETECTOR_H
#define HORIZONLINEDETECTOR_H

class Node
{
    public:
        Node(){}
        Node(std::shared_ptr<Node> p,int x_,int y_){prev=p;x=x_;y=y_;lost=0;}
        Node(Node &n1){x=n1.x;y=n1.y;cost=n1.cost;lost=n1.lost;prev=n1.prev;}//next=n1.next;}
        std::shared_ptr<Node> prev;             //Previous and next nodes
        //std::vector<std::shared_ptr<Node> > next;
        int cost;               //Total cost until now
        int lost;               //Number of steps taken without a clear path
        int x,y;
        Node& operator=(const Node &n1){x=n1.x;y=n1.y;cost=n1.cost;lost=n1.lost;prev=n1.prev;}//next=n1.next;}
        Node& operator=(Node &&n1){x=n1.x;y=n1.y;cost=n1.cost;lost=n1.lost;prev=n1.prev;n1.prev=nullptr;}//next=n1.next;n1.next.clear();}
};

class HorizonLineDetector
{
	private:
		//Simple edge computation
        void compute_edges(const cv::Mat &mask=cv::Mat());
		//Dynamic programming path solver
		void compute_dp_paths();
		//Compute features on edge locations
        void compute_descriptors();
        //Find edge locations and add them to list
        void find_edge_list(const cv::Mat& mask=cv::Mat());
        void reset_nodes();
        void delete_nodes();
		//List of found edges
        bool compute_cheapest_path(const cv::Mat &mask=cv::Mat());
        bool dp(std::shared_ptr<Node> n);
        void reset_dp();
        void add_node_to_horizon(std::shared_ptr<Node> n);
        cv::Ptr<cv::DescriptorExtractor> extractor;
        //cv::OrbDescriptorExtractor extractor;
        std::vector<cv::KeyPoint> current_keypoints;
        cv::Mat current_frame,current_edges, current_edges_list,current_draw;
        cv::Mat trainingDataMat, labelsMat,descriptorsMat;
        int canny_param=30;
        cv::Ptr<cv::ml::SVM> svm;
        std::vector<bool> valid_edges;
        bool load_model(const std::string config_file);
        int max_lost_steps=10;
        std::shared_ptr<Node> first_node, last_node;
        cv::Mat_<int> visited;
        std::vector<cv::Point2i> horizon;
        std::multimap<int,std::shared_ptr<Node>> ntree;                       //Structure to keep track of the leafs (ordered)
        //std::list<std::shared_ptr<Node>> nlist;
        int lost_step_cost=5;
        int max_cost=2;
	public:
        HorizonLineDetector();
		~HorizonLineDetector(){}
		//Initialize training mode
        bool train(const std::string training_list_file);
		//Initialize detecting mode
		bool init_detector(const std::string training_config_file);
        //Save and load training
        bool save_model(const std::string config_file);
        void set_max_lost_steps(const int max_lost){max_lost_steps=max_lost;}
        int get_max_lost_steps(){return max_lost_steps;}
        void draw_horizon();
        void draw_edges();
        int get_cany_param(){return canny_param;}
        //A low canny_param will detect many edges therefore the HL will take longer to compute.
        //A high value might miss some of the edges in the image
        void set_canny_param(const int cp){canny_param=cp;}
        int get_max_search_steps(){return max_lost_steps;}
        void set_max_search_steps(const int mss){max_lost_steps=mss;}
		//Detect horizon line in image
        void detect_image(const cv::Mat &frame, const cv::Mat &mask=cv::Mat());
		//Detect horizon file in video
        bool detect_video(const std::string video_file, const std::string video_file_out, const cv::Mat &mask_=cv::Mat());
        void save_draw_frame(const std::string file_name="draw.jpg");
};

#endif //HORIZONLINEDETECTOR_H
