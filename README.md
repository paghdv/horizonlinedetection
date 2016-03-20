# horizonlinedetection
Horizon line detection

This is an implementation of a relatively basic horizon line detection based on dynamip programming and learning. 
There are 3 different weights for each node (pixel), low (point detected as potential horizon line), medium (edge), and high (any other).
Ideally the entire horizon line will be detected by the features and the path will be found quickly. In many cases the edges in the horizon lines are not correctly detected as horizon line so an edge can be take. In few caes the horizon line is not even seen as an edge so we need to explore the area but there's a maximum explorarion steps that could be taken.

The algorithm is very similar to:"A Machine Learning Approach to Horizon Line Detection Using Local Features", T Ahmad, G Bebis, EE Regentova, A Nefian - Advances in Visual Computing, 2013

Usage for train: ./HorizonLineDetector 0 file_path_in training_file <br/>
Usage for run  : ./HorizonLineDetector 1 file_path_in file_path_out <br/>
       		  file_path_in will be a list file in the case of training and a video file path in the case of running<br/>
		  file_path_out video file path<br/>
		  training_file will config file output in the case of training and will determine the file to use when running<br/>


<a href="http://www.youtube.com/watch?feature=player_embedded&v=HG2LBKFSUjk
" target="_blank"><img src="http://img.youtube.com/vi/HG2LBKFSUjk/0.jpg" 
alt="Example" width="240" height="180" border="10" /></a> <a href="http://www.youtube.com/watch?feature=player_embedded&v=6PyR47v7BmY
" target="_blank"><img src="http://img.youtube.com/vi/6PyR47v7BmY/0.jpg" 
alt="Example" width="240" height="180" border="10" /></a>



