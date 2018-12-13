#ifndef __IPM_H__
#define __IPM_H__

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

class IPM
{
public:	
	IPM( const cv::Size& _origSize, const cv::Size& _dstSize, 
		const std::vector<cv::Point2f>& _origPoints, const std::vector<cv::Point2f>& _dstPoints );		
	
	// Apply IPM on points
	cv::Point2d applyHomography(const cv::Point2d& _point, const cv::Mat& _H);
	void applyHomography( const cv::Mat& _origBGR, cv::Mat& _ipmBGR, int borderMode = cv::BORDER_CONSTANT);

private:
	void createMaps();

	// Sizes
	cv::Size m_origSize;
	cv::Size m_dstSize;

	// Points	
	std::vector<cv::Point2f> m_origPoints;
	std::vector<cv::Point2f> m_dstPoints;

	// Homography
	cv::Mat m_H;
	cv::Mat m_H_inv;

	// Maps	
	cv::Mat m_mapX, m_mapY;
};

#endif /*__IPM_H__*/