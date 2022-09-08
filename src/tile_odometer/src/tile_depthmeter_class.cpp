#include "tile_depthmeter/tile_depthmeter_class.hpp"

using namespace std; 
using namespace cv;

TileDepthmeter::TileDepthmeter() {
}

void TileDepthmeter::SetTileSize(float heigth, float width) { 
    pTile.height = heigth;
    pTile.width = width;
}

void TileDepthmeter::SetCameraIntrinsics(vector<double> intr_vals) {
    pCam.fx = intr_vals[0];
    pCam.cx = intr_vals[2];
    pCam.fy = intr_vals[4];
    pCam.cy = intr_vals[5];
}

void TileDepthmeter::SetFilterParams(int sample_size, int cycles, float thrshld, int min_inliers, float min_rect) {
    pFltr.sample_size = sample_size; 
    pFltr.iterations = cycles;
    pFltr.threshold = thrshld;
    pFltr.min_score = min_inliers;
    pFltr.min_rect_size = min_rect; 
}

void TileDepthmeter::SetPreprocessingParams(Size kernel, int laplace_krnl, int bin_thrshld, int erosion, int dilution) {
    pProc.gaussian_kernel = kernel;
    pProc.laplacian_kernel = laplace_krnl;
    pProc.bin_threshold = bin_thrshld;
    pProc.erosion = erosion; 
    pProc.erosion = dilution;
}

void TileDepthmeter::SetFrame(Mat new_frame) {
    frame = new_frame; 
}

Mat TileDepthmeter::GetFrame() {
    return frame;  
}

vector<RotatedRect> TileDepthmeter::GetFilteredRects(){
    return filtered_rects; 
}

float TileDepthmeter::CalcDistance() {
    scale_factor_ = CalcScale();
    float distance = pCam.fx*scale_factor_;
    return distance;
}

float TileDepthmeter::CalcScale() {
    FindRectangles();

    FindDominantRects();

    float area_perceived = FindAverageArea(filtered_rects);
    float area_real = pTile.height*pTile.width;
    scale_factor_ = sqrt(area_real / area_perceived);

    return scale_factor_; 
}

void TileDepthmeter::PreprocessImage() {
    cvtColor(frame, frame, COLOR_BGR2GRAY);

    GaussianBlur(frame, frame, pProc.gaussian_kernel, 0, 0);

    Laplacian(frame, frame, CV_8U, pProc.laplacian_kernel, 1, 0);

    threshold(frame, frame, pProc.bin_threshold, 255, THRESH_BINARY_INV);

    erode(frame, frame, Mat(), Point(-1, -1), pProc.erosion, 1, 1);

    dilate(frame, frame, Mat(), Point(-1, -1), pProc.dilution, 1, 1); 
}

void TileDepthmeter::FindRectangles() {
    vector<vector<Point> > contours;
    findContours(frame, contours, RETR_LIST, CHAIN_APPROX_NONE);

    vector<vector<Point> >hull(contours.size());
    for(int i = 0; i < contours.size(); i++) {
        convexHull(contours[i], hull[i]);
    }
    contours = hull;

    raw_rects = vector<RotatedRect>(contours.size(), RotatedRect(Point2f(0, 0), Point2f(0, 0), Point2f(0, 0)));
    int rect_cntr = 0; 
    for(int i = 0; i < contours.size(); i++) {
        if(minAreaRect(contours[i]).size.area() > pFltr.min_rect_size) {
            raw_rects[rect_cntr] = minAreaRect(contours[i]);
            rect_cntr++;
        }
    }
    raw_rects.erase(raw_rects.begin() + rect_cntr, raw_rects.end());
}

void TileDepthmeter::FindDominantRects() {
    int score = 0; 
    float area = 0; 
    for(int i = 0; i < raw_rects.size(); i++){
        vector<RotatedRect> inlier_rects; 
        int points_cntr = 0;
        for(int j = i; j < raw_rects.size(); j++){
            if(abs(raw_rects[i].size.area() - raw_rects[j].size.area()) < pFltr.threshold) {
                points_cntr++; 
                inlier_rects.push_back(raw_rects[j]); 
            } 
        }
        if(points_cntr > pFltr.min_score && area < FindAverageArea(inlier_rects)){
            area = FindAverageArea(inlier_rects); 
            score = points_cntr; 
            filtered_rects = inlier_rects; 
        }
    }
}

float TileDepthmeter::FindAverageArea(vector<RotatedRect> rectangles){
    float area = 0; 
    for(int i = 0; i < rectangles.size(); i++){
        area = rectangles[i].size.area(); 
    }
    area /= rectangles.size(); 

    return area; 
}