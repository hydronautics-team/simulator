#include "monocular_odometry/pose_estimator_class.hpp"

using namespace std;

void PoseEstimator::SetPrevFrame(Mat frame){
    frame_next = frame; 
    SharpenImage();
    SquareImage();
    CycleData(); 
}

void PoseEstimator::SetNextFrame(Mat frame){
    frame_next = frame; 
    SharpenImage();
    SquareImage(); 
}

void PoseEstimator::SetGaussParams(int stype, int dtype, Size k, float s_x, float s_y){
    pGau.src_type = stype; 
    pGau.dst_type = dtype; 
    pGau.ksize = k;
    pGau.sigma_x = s_x;
    pGau.sigma_y = s_y; 
}

void PoseEstimator::SetLaplaceParams(int stype, int dtype, int k){
    pLap.src_type = stype; 
    pLap.dst_type = dtype; 
    pLap.ksize = k; 
    
}

void PoseEstimator::SetMatcherParams(int number_of_cells, int border_size, int max_spread, int stype, int m, Size ks){
    pMat.number_of_cells = number_of_cells; 
    pMat.border_size = border_size; 
    pMat.number_of_cells = number_of_cells;
    pMat.src_type = stype; 
    pMat.method = m;
    pMat.size = ks; 
}

void PoseEstimator::SetRMatFromQ(vector<double> quat){
    R = Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();
}

Mat PoseEstimator::GetProcessedFrame(){
    Mat frame; 
    gpu_frame_next.download(frame);
    return frame; 
}

vector<double> PoseEstimator::GetYPR(){
    vector<double> temp{ YPR[0],
                        YPR[1],
                        YPR[2]};
    return temp; 
}

vector<double> PoseEstimator::GetQ(){
    vector<double> temp{Q.w(),
                        Q.x(),
                        Q.y(),
                        Q.z()};
    return temp; 
}

vector<double> PoseEstimator::GetT(){
    vector<double> temp{T[0],
                        T[1],
                        T[2]};
    return temp; 
}

bool PoseEstimator::CalculateRotation(){
    SetMatrix(); 
    ExtractMotion(); 
    FilterDeltas(); 

    if(deltas_filtered.rows() < 3){
        CycleData();
        return false; 
    }
    
    Convert3D();
    CalcNextCentroid();
    CenterSet();
    ExtractRotation();
    CheckRCase(); 
    Convert2YPR(); 
    Convert2Q(); 

    CycleData();
    return true; 
}

bool PoseEstimator::CalculateTranslation(){
    T = centroid_next - R*centroid_prev; 
    return true; 
}

void PoseEstimator::SharpenImage(){
    gpu_frame_next.upload(frame_next); 
    gpu_frame_mask.upload(frame_next); 

    cuda::cvtColor(gpu_frame_next, gpu_frame_next, COLOR_BGR2GRAY);
    cuda::cvtColor(gpu_frame_mask, gpu_frame_mask, COLOR_BGR2GRAY);

    gpu_frame_next.convertTo(gpu_frame_next, CV_8U);
    gpu_frame_mask.convertTo(gpu_frame_mask, CV_8U);

    cuda::normalize(gpu_frame_next, gpu_frame_next, 0, pow(2, 8), NORM_MINMAX, CV_8U);
    cuda::normalize(gpu_frame_mask, gpu_frame_mask, 0, pow(2, 8), NORM_MINMAX, CV_8U);

    gaussian_filter->apply(gpu_frame_mask, gpu_frame_mask);
    laplacian_filter->apply(gpu_frame_mask, gpu_frame_mask); 

    cuda::subtract(gpu_frame_next, gpu_frame_mask, gpu_frame_next);
}

void PoseEstimator::SetMatrix(){
    int rows = sqrt(pMat.number_of_cells); 
    int cols = sqrt(pMat.number_of_cells);
    if(pMat.number_of_cells%2 == 0){
        chosen_cells = MatrixXi(4*2, 2);
        chosen_cells << 0,         cols/2-1, 
                        0,         cols/2, 
                        rows/2-1,  0, 
                        rows/2-1,  cols-1, 
                        rows/2,    0, 
                        rows/2,    cols-1,
                        rows-1,    cols/2-1, 
                        rows-1,    cols/2;
    }else{
        chosen_cells = MatrixXi(4, 2);
        chosen_cells << 0,             (int)cols/2, 
                        (int)rows/2,   0, 
                        (int)rows/2,   cols-1, 
                        rows-1,        (int)cols/2;
    }

}

void PoseEstimator::SquareImage(){
    int shift = (frame_next.cols - frame_next.rows)/2; 
    Rect square_image(shift, 0, frame_next.rows, frame_next.rows);
    gpu_frame_next = gpu_frame_next(square_image); 
}

void PoseEstimator::ExtractMotion(){
    deltas = MatrixX2d(chosen_cells.rows(), 2);
    positions = MatrixX2d(chosen_cells.rows(), 2);

    int chosen_cells_counter = 0; 
    int cell_size = frame_next.rows/sqrt(pMat.number_of_cells);
    for(int i = 0; i < sqrt(pMat.number_of_cells); i++){
        for(int j = 0; j < sqrt(pMat.number_of_cells); j++){
            if(chosen_cells.row(chosen_cells_counter)[0] == i &&  chosen_cells.row(chosen_cells_counter)[1] == j){
                chosen_cells_counter++;
            }else{
                continue;
            }
            int subregion_shift_x = pMat.border_size + cell_size * i;
            int subregion_shift_y = pMat.border_size + cell_size * j;
            Rect subregion_to_match(subregion_shift_x, subregion_shift_y, cell_size - 2*pMat.border_size, cell_size - 2*pMat.border_size); 
            gpu_subregion_to_match = gpu_frame_prev(subregion_to_match);

            int region_shift_x = cell_size * i;
            int region_shift_y = cell_size * j;
            Rect region_for_matching(region_shift_x, region_shift_y, cell_size, cell_size);
            gpu_region_for_matching = gpu_frame_next(region_for_matching);
            matcher->match(gpu_region_for_matching, gpu_subregion_to_match, gpu_matching_results);
            
            gpu_matching_results.download(matching_results); 
            double minVal, maxVal;
            Point minLoc, maxLoc;
            minMaxLoc(matching_results, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

            deltas.row(chosen_cells_counter-1) << maxLoc.x - pMat.border_size, maxLoc.y - pMat.border_size; 
            positions.row(chosen_cells_counter-1) << subregion_shift_x, subregion_shift_y; 

            if(chosen_cells_counter == chosen_cells.rows()){
                return; 
            }
        }
    }
}

void PoseEstimator::FilterDeltas(){
        int best_score = 0; 
        float max_distance = 2; 
        deltas_filtered = MatrixXd(0, 2); 
        positions_filtered = MatrixXd(0, 2); 
        for(int i = 0; i < deltas.rows(); i++){
            for(int j = i+1; j < deltas.rows(); j++){
                Vector2d centroid; 
                centroid << 0, 0; 
                centroid = (deltas.row(i) + deltas.row(j))/2;

                int score = 0;
                MatrixXd deltas_inliers(0, 2);
                MatrixXd positions_inliers(0, 2);
                for(int k = 0; k < deltas.rows(); k++){
                    if((deltas.row(k)-centroid.transpose()).norm() < max_distance){
                        score++; 
                        deltas_inliers.conservativeResize(deltas_inliers.rows()+1, NoChange);
                        deltas_inliers.row(deltas_inliers.rows()-1) = deltas.row(k); 

                        positions_inliers.conservativeResize(positions_inliers.rows()+1, NoChange);
                        positions_inliers.row(positions_inliers.rows()-1) = positions.row(k); 
                    }

                    if(score > best_score){
                        deltas_filtered = deltas_inliers; 
                        positions_filtered = positions_inliers; 
                    }
                }
            }
        }
}

void PoseEstimator::Convert3D(){
    points3d_prev = MatrixXd(deltas_filtered.rows(), 3); 
    points3d_next = MatrixXd(deltas_filtered.rows(), 3); 
    for(int i = 0; i < deltas_filtered.rows(); i++){
        points3d_prev.row(i) << positions_filtered.row(i)[0],
                                positions_filtered.row(i)[1],
                                1; 
        points3d_next.row(i) << deltas_filtered.row(i)[0] + positions_filtered.row(i)[0],
                                deltas_filtered.row(i)[1] + positions_filtered.row(i)[1],
                                1; 
    }
}

void PoseEstimator::CalcNextCentroid(){
    centroid_prev = points3d_prev.colwise().mean();
    centroid_next = points3d_next.colwise().mean(); 
}

void PoseEstimator::CenterSet(){
    points3d_prev = (-points3d_prev).rowwise() + centroid_prev.transpose(); 
    points3d_next = (-points3d_next).rowwise() + centroid_next.transpose(); 
}

void PoseEstimator::ExtractRotation(){
    H = points3d_prev.transpose() * points3d_next; 

    JacobiSVD<MatrixXd> svd(H, ComputeFullV | ComputeFullU);

    R = svd.matrixV() * svd.matrixU().transpose(); 
}

void PoseEstimator::CheckRCase(){
    if(R.determinant() < 0){ 
        JacobiSVD<MatrixXd> svd(R, ComputeFullV | ComputeFullU);
        MatrixXd Vt = svd.matrixV(); 
        Vector3d M = Vector3d(1, 1, -1);
        Matrix3Xd C = Vt.array().colwise() * M.array();
        R =  Vt * svd.matrixU().transpose();
    }
} 

void PoseEstimator::Convert2YPR(){
        YPR = R.eulerAngles(2, 1, 0) * 360/CV_PI; 
        for(int i = 0; i < YPR.size(); i++)
            if(YPR[i] > 180)
                YPR[i] -= 360; 
}

void PoseEstimator::Convert2Q(){
    Q = R; 
}

void PoseEstimator::CycleData(){
    frame_prev = frame_next;
    gpu_frame_prev = gpu_frame_next;
}