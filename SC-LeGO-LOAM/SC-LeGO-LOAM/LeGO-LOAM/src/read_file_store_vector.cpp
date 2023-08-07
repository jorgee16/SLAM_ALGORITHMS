#include "read_file_store_vector.h"

bool LOOPS_FROM_FILE::has_loops(std::vector <int> frame){
    if (frame[0] < 0) return false;
    else return true;
}
void LOOPS_FROM_FILE::read_file(std::string filename1,std::string filename2){
    // std::vector <std::vector<int>> LOOPS_INDEX;
    // std::vector <std::vector<float>> SIMILARITY_SCORE;

    std::ifstream file1,file2;
    // std::string filename1,filename2;
    frame_id = 0;

    // int fill = 0;
    // std::vector<int> fill_int;
    // std::vector <float> fill_float;

    // fill_int.push_back(-1);
    // fill_float.push_back(-1);

    // while(fill < 3){

    //      LOOPS_INDEX.push_back(fill_int);
    //      SIMILARITY_SCORE.push_back(fill_float);
    //      fill++;
    // }

    // filename1 = "/home/joaojorge/Documents/relocalization/VLAD/prediction.txt";
    // filename2 = "/home/joaojorge/Documents/relocalization/VLAD/similarity.txt";

    file1.open(filename1,std::ios::in);
    file2.open(filename2,std::ios::in);

    if (!file1.is_open()){
        std::cout << ("Error opening file.\n");
        
    }
    if (!file2.is_open()){
        std::cout << ("Error opening file.\n");
    }

    std::string line;
    while(getline(file1,line))
    {       
            if (line.empty())
            {
                std::vector <int> coluna;
                coluna.push_back(-1);
                LOOPS_INDEX.push_back(coluna);
            }
            
            else
            {
                std::vector<int> coluna;
                std::istringstream iss(line);
                int val;
                while( iss >> val){
                    coluna.push_back(val);
                }

                LOOPS_INDEX.push_back(coluna);     
            }
    }

    std::string line2;
    while(getline(file2,line2))
    {
        if (line2.empty())
        {
            std::vector <float> coluna;
            coluna.push_back(-1);
            SIMILARITY_SCORE.push_back(coluna);
        }
        
        else
        {
            std::vector<float> coluna;
            std::istringstream iss(line2);
            float val;
            while( iss >> val){
                coluna.push_back(val);
            }

            SIMILARITY_SCORE.push_back(coluna);    
        }
    }
    file1.close();
    file2.close();

}

int LOOPS_FROM_FILE::indexes (std::vector<std::vector<int>> index, int frame_id){
    // std::vector<int> candidate_indexes;
    
    //             for (int j = 0; j < index[frame_id].size(); j++)
    //             {
    //                 candidate_indexes.push_back(index[frame_id][j]);
    //             }
    int candidate_idx;
    candidate_idx = index[frame_id][0];

    return candidate_idx;
}

float LOOPS_FROM_FILE::scores (std::vector<std::vector<float>> score, int frame_id){
    // std::vector<float> candidate_scores;
    
    //             for (int j = 0; j < score[frame_id].size(); j++)
    //             {
    //                 candidate_scores.push_back(score[frame_id][j]);
    //             }
    float candidate_score;
    candidate_score = score[frame_id][0];

    return candidate_score;
}

void LOOPS_FROM_FILE::makeAndSaveScancontextAndKeys( pcl::PointCloud<pcl::PointXYZI> & _scan_down ){
    frame_id++;
    return;
}


std::pair<int, float> LOOPS_FROM_FILE::detectLoopClosureID(){

     int loop_id { -1 }; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")
    /* 
     * step 1: candidates from ringkey tree_
     */

    std::cout << std::endl << "frame_id:" << frame_id << std::endl;
    // std::cout << "frame_id: " << frame_id << std::endl;
    if(LOOPS_FROM_FILE::has_loops(LOOPS_INDEX[frame_id]) == false)
    {
        // std::pair<int, float> result {loop_id, 0.0};
        // return result; // Early return 
        return std::make_pair(-1,0.0);
    }
    // std::vector<int> candidates_cur_frame;
    // std::vector <float> scores_cur_frame;

    float min_score;
    int nn_idx;

    nn_idx = LOOPS_FROM_FILE::indexes(LOOPS_INDEX,frame_id);
    min_score = LOOPS_FROM_FILE::scores(SIMILARITY_SCORE, frame_id);

    int counter = 0;

    for (unsigned long i = 0; i < LOOP_BUFFER_GLOBAL.size(); i++){ 
        if (nn_idx == LOOP_BUFFER_GLOBAL[i]){

            if(min_score < 0.35){
                std::cout.precision(3); 
                std::cout << "[Loop found] Minimum Score: " << min_score << " between " << frame_id << " and " << nn_idx << "." << std::endl;
                
                return std::make_pair(LOOP_BUFFER_FILTERED[counter],min_score);
            }
            else{
                std::cout.precision(3); 
                std::cout << "[Not loop] Minimum Score: " << min_score << " between " << frame_id << " and " << nn_idx << "." << std::endl;
                return std::make_pair(-1,min_score);
            }
                
        }
        counter++;
            // std::cout << "Counter: " << counter << std::endl;
    }
    

    // std::pair<int, float> result {nn_idx, min_score};
    // return result;
    return std::make_pair(-1,0.0);
}

int LOOPS_FROM_FILE::checkBUFFER(int nn_idx, float min_score){

    int counter = 0;
    if (nn_idx < 0) 
        return (-1);

    else {
        for (unsigned long i = 0; i < LOOP_BUFFER_GLOBAL.size(); i++){ 
            if (nn_idx == LOOP_BUFFER_GLOBAL[i]){
                // std::cout << std::endl;
                if(min_score < 0.35){
                    std::cout.precision(3); 
                    std::cout << "[Loop found] Minimum Score: " << min_score << " between " << frame_id << " and " << nn_idx << "." << std::endl;
                    
                    return LOOP_BUFFER_FILTERED[counter];
                }
                else{
                    std::cout.precision(3); 
                    std::cout << "[Not loop] Minimum Score: " << min_score << " between " << frame_id << " and " << nn_idx << "." << std::endl;
                    return (-1);
                }
                
            }
            counter++;
            // std::cout << "Counter: " << counter << std::endl;
        }
    }
    return (-1);
}