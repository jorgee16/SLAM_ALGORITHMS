// #pragma once

#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_map>
#include <chrono>

// using INDEX = std::vector<std::vector<int>>;
// using SCORE = std::vector<std::vector<float>>;


class LOOPS_FROM_FILE
{
        
        // INDEX LOOPS_INDEX;
        // SCORE SIMILARITY_SCORE;

    public:

        std::vector<std::vector<int>> LOOPS_INDEX;
        std::vector<std::vector<float>> SIMILARITY_SCORE;
        std::vector<std::vector<int>> LOOPS_INDEX_GT;
        
        int frame_id;
        std::vector<int> LOOP_BUFFER_GLOBAL;
        std::vector<int> LOOP_BUFFER_FILTERED;

        std::vector<int> GT_INDEX;
        std::vector<int> INDEX;
        std::vector<int> ICP_INDEX;

        bool has_loops(std::vector <int> frame);
        void read_file(std::string filename1,std::string filename2);
        void read_GT(std::string filename);

        void makeAndSaveScancontextAndKeys( pcl::PointCloud<pcl::PointXYZI> & _scan_down );
        int indexes(std::vector<std::vector<int>> index, int cur_frame_id);
        float scores(std::vector<std::vector<float>> score, int cur_frame_id);
        std::pair<int, float> detectLoopClosureID(); // int: nearest node index, float: relative yaw  
        // int checkBUFFER(int loop_id, float min_score);

        void store_indexes(int idx, int ICP_idx);
        void save_indexes();
        void save_ICP_INDEX();
        

        // LOOPS_FROM_FILE():
        // {
        //     LOOPS_FROM_FILE(read_file());  
        // }
        
};
