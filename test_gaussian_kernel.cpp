#include "RadarModel.hpp"

#include <cmath>
#include <iostream>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// #include "Eigen/Eigen"
// #include <Eigen/Core>



using namespace std;
using namespace cv;

Eigen::MatrixXf getGaussianKernel(int rows, int cols, int x_mean, int y_mean, double sigmax, double sigmay)
{
    // const auto y_mid = (rows-1) / 2.0;
    // const auto x_mid = (cols-1) / 2.0;

    const auto y_mid = y_mean;
    const auto x_mid = x_mean;

    const auto x_spread = 1. / (sigmax*sigmax*2);
    const auto y_spread = 1. / (sigmay*sigmay*2);

    const auto denominator = 8 * std::atan(1) * sigmax * sigmay;

    std::vector<double> gauss_x, gauss_y;

    gauss_x.reserve(cols);
    for (auto i = 0;  i < cols;  ++i) {
        auto x = i - x_mid;
        gauss_x.push_back(std::exp(-x*x * x_spread));
    }

    gauss_y.reserve(rows);
    for (auto i = 0;  i < rows;  ++i) {
        auto y = i - y_mid;
        gauss_y.push_back(std::exp(-y*y * y_spread));
    }

    Eigen::MatrixXf kernel = Eigen::MatrixXf::Zero(rows, cols);
    for (auto j = 0;  j < rows;  ++j)
        for (auto i = 0;  i < cols;  ++i) {
            kernel(j,i) = gauss_x[i] * gauss_y[j] / denominator;
        }

    return kernel;
}

int main(int argc, char **argv){
    int max_iteration = 20;
    double sigma = 2;
    int W = 200;
    // double kernel[W][W];
    cv::Mat kernel_mat = cv::Mat::zeros(W, W, CV_32F);
    Eigen::MatrixXf kernel;
    double mean_x = 7;
    double mean_y = 40;
    double sum = 0.0;

    for(int i=0; i<max_iteration; i++){
        sum = 0.0; // For accumulating the kernel values
        // Increase the variance at every iteration
        if (i !=0 ) sigma = sigma + 2;

        // for (int x = 0; x < W; ++x) 
        //     for (int y = 0; y < W; ++y) {
        //         kernel.at<float>(x,y) = exp( -0.5 * (pow((x-mean)/sigma, 2.0) + pow((y-mean)/sigma,2.0)) )
        //                         / (2 * M_PI * sigma * sigma);

        //         // Accumulate the kernel values
        //         sum += kernel.at<float>(x,y);
        //     }

        // // Normalize the kernel
        // for (int x = 0; x < W; ++x) 
        //     for (int y = 0; y < W; ++y)
        //         kernel.at<float>(x,y) /= sum;

        kernel = getGaussianKernel(W, W, mean_x, mean_y, sigma, sigma);

        // Print the kernel
        // for(int i = 0; i < 10; ++i)
        // {
        //     for (int j = 0; j < 10; ++j)
        //         cout<<kernel.at<float>(i,j)<<"\t";
        //     cout<<endl;
        // }

        // Show image in a window but first normalize in 0-255
        double minValue, maxValue;
        Point minLoc, maxLoc;
        // cv::minMaxLoc(kernel, &minValue, &maxValue, &minLoc, &maxLoc);
        // kernel = (kernel - minValue)/(maxValue - minValue);
        kernel = kernel  / kernel.maxCoeff();
        kernel = 255*kernel;
        // eigen2cv(kernel, kernel_mat);
        // Naming the window 
        String geek_window = "MY SAVED IMAGE"; 
        // Creating a window 
        cv::namedWindow(geek_window); 
        // cv::Mat_<float> a = Mat_<float>::ones(W,W);
        // cv::cv2eigen(a, kernel);
        cv::eigen2cv(kernel, kernel_mat);
        // Showing the image in the defined window 
        cv::imshow(geek_window, kernel_mat); 
        // waiting for any key to be pressed 
        cv::waitKey(0); 
        // destroying the creating window 
        cv::destroyWindow(geek_window); 
    }
    

    // Save on disk but first normalize in 0-255
    bool check = cv::imwrite("/home/pulver/Desktop/gaussian_kernel.jpg", kernel_mat);

}