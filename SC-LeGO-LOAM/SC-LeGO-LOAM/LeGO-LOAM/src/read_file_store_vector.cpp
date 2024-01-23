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

void LOOPS_FROM_FILE::read_GT(std::string filename){

    std::ifstream file1;

    file1.open(filename,std::ios::in);

    if (!file1.is_open()){
        std::cout << ("Error opening file.\n");
        
    }
 
    std::string line;
    while(getline(file1,line))
    {       
            if (line.empty())
            {
                std::vector <int> coluna;
                coluna.push_back(-1);
                LOOPS_INDEX_GT.push_back(coluna);
            }
            
            else
            {
                std::vector<int> coluna;
                std::istringstream iss(line);
                int val;
                while( iss >> val){
                    coluna.push_back(val);
                }

                LOOPS_INDEX_GT.push_back(coluna);     
            }
    }
    file1.close();

}

std::vector<int> LOOPS_FROM_FILE::indexes (std::vector<std::vector<int>> index, int frame_id){
    std::vector<int> candidate_indexes;
    // index[frame_id].size()
    // store the first 5 indexes
    for (int j = 0; j < 5; j++)
    {
        candidate_indexes.push_back(index[frame_id][j]);
    }
    // int candidate_idx;
    // candidate_idx = index[frame_id][0];

    return candidate_indexes;
}

std::vector<float> LOOPS_FROM_FILE::scores (std::vector<std::vector<float>> score, int frame_id){
    std::vector<float> candidate_scores;
    // score[frame_id].size()
    // store the first 5 scores em vez de todos 
    for(int j = 0; j < 5; j++)  
    {
        candidate_scores.push_back(score[frame_id][j]);
    }
    // float candidate_score;
    // candidate_score = score[frame_id][0];

    return candidate_scores;
}

// void LOOPS_FROM_FILE::makeAndSaveScancontextAndKeys( pcl::PointCloud<pcl::PointXYZI> & _scan_down ){
//     frame_id++;
//     return;
// }


std::pair<int, float> LOOPS_FROM_FILE::detectLoopClosureID(){

    std::vector <float> min_score;
    std::vector <int> nn_idx;
    int loop_id { -1 }; 

    // std::cout << "FRAME ID: " << frame_id << std::endl;

    if(LOOPS_FROM_FILE::has_loops(LOOPS_INDEX[frame_id]) == false)
    {
        min_score = LOOPS_FROM_FILE::scores(SIMILARITY_SCORE, frame_id);
        return std::make_pair(-1,min_score[0]);
    }

    
   
    nn_idx = LOOPS_FROM_FILE::indexes(LOOPS_INDEX,frame_id);
    min_score = LOOPS_FROM_FILE::scores(SIMILARITY_SCORE, frame_id);

    // int counter = 0;

    for (unsigned long i = 0; i < LOOP_BUFFER_GLOBAL.size(); i++){ 
        for(int j= 0; j < nn_idx.size(); j++){ 
            if (nn_idx[j] == LOOP_BUFFER_GLOBAL[i]){

                if(min_score[j] < 0.5){
                    std::cout.precision(3); 
                    std::cout << "[Loop found] Minimum Score: " << min_score[j] << " between " << frame_id << " and " << nn_idx[j] << "." << std::endl;
                    
                    return std::make_pair(LOOP_BUFFER_FILTERED[i],min_score[j]);
                }
                else{
                    std::cout.precision(3); 
                    std::cout << "[Not loop] Minimum Score: " << min_score[j] << " between " << frame_id << " and " << nn_idx[j] << "." << std::endl;
                    return std::make_pair(-1,min_score[j]);
                }
                    
            }
        }
        // counter++;
    }

    // std::pair<int, float> result {nn_idx, min_score};
    // return result;
    return std::make_pair(-1,min_score[0]);
}

void LOOPS_FROM_FILE::store_indexes(int idx, int ICP_idx){

    std::vector<int> gt_index_glob;
    int gt_index_loc;
    int contador = 0;
    // std::cout << LOOP_BUFFER_FILTERED[idx] << '\n';
    // std::cout << "storing indexes" << std::endl;
    // std::cout <<  "TAMANHO DE GT: " << LOOPS_INDEX_GT.size() << std::endl;

    INDEX.push_back(idx);
    
    std::cout << "INDEX: " << idx << std::endl;
    std::cout << "ICP_INDEX: " << ICP_idx << std::endl;

    // if(LOOPS_FROM_FILE::has_loops(LOOPS_INDEX_GT[frame_id]) == false)
    // {
    //     GT_INDEX.push_back(-1);
    //     std::cout << "GT_INDEX: " << "-1" << std::endl;
    //     return;
    // }
    // else{

    //     gt_index_glob = LOOPS_FROM_FILE::indexes(LOOPS_INDEX_GT, frame_id);


    //     for (int i = 0; i < LOOP_BUFFER_GLOBAL.size(); i++){
    //         for(int j = 0; j < gt_index_glob.size(); j++){
    //             if (gt_index_glob[j] == LOOP_BUFFER_GLOBAL[i]){

    //                 GT_INDEX.push_back(LOOP_BUFFER_FILTERED[i]);
    //                 std::cout << "GT_INDEX: " << LOOP_BUFFER_FILTERED[i] << std::endl;
    //                 return;
    //             }
    //         }
    //         // contador++;
    //     }
    //     GT_INDEX.push_back(-1);
    //     return;
    // }
}

void LOOPS_FROM_FILE::save_indexes(float min_dist, double feature_dist){
        std::ofstream outputFile("/home/joaojorge/INDEXES.txt", std::ofstream::app);

        // std::cout << "Tamanho do SC INDEX: " << loops_rede.INDEX.size()
        if (outputFile.is_open()){
            // Write ground truth and scan context output indexes to the file
            // std::cout << "TAMANHO GT_INDEX: " << GT_INDEX.size() << std::endl;
            // std::cout << "TAMANHO DE ICP_INDEX:" << ICP_INDEX.size() << std::endl;

            // for (int i = 0; i < GT_INDEX.size() && i < INDEX.size(); ++i) {
            //     outputFile << GT_INDEX[i] << " " << INDEX[i] << '\n';
            // }

            // Grava o último valor dos vectores INDEX e GT_INDEX
            if (!GT_INDEX.empty() || !ICP_INDEX.empty()) {
                outputFile << INDEX.back() << " " << ICP_INDEX.back() << " " << min_dist << " " << feature_dist << '\n';
                // << " " << ICP_INDEX.back()
            } else {
                outputFile << "GT_INDEX está vazio.\n";

            outputFile.close();
            std::cout << "Data written to the file successfully." << std::endl;
            } 
        }
        else{
            std::cout << "Error opening the file." << std::endl;
        }
        
    return;
}