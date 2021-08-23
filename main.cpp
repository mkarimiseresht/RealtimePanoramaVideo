#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"

#include <stdio.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <fstream>

#define NUM_OF_CAM 2

using namespace cv;
using namespace std;
//using namespace cv::detail;


int main(int argc, char *argv[])
{

	//int const FRAME_WIDTH = 176;
	//int const FRAME_HEIGHT = 144;

	int const FRAME_WIDTH = 160*3;
	int const FRAME_HEIGHT = 120*3;



	//*
	VideoCapture cap1(1);
	if (!cap1.isOpened())
		return -1;

	cap1.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap1.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	cap1.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	//*/
	VideoCapture cap2(2);
	cap2.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	if (!cap2.isOpened())
		return -1;



	cap2.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	cap2.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	/*
	VideoCapture cap3(2);
	cap3.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));
	if (!cap3.isOpened())
	return -1;


	cap3.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	cap3.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	*/


	//Mat pano_result;
	Mat img;
	Mat pano_fin;
	float blend_strength = 5;
	// Features
	Ptr<detail::FeaturesFinder> finder;
	finder = new detail::SurfFeaturesFinder();
	// or : finder = new SurfFeaturesFinderGPU();
	vector<detail::ImageFeatures> features(NUM_OF_CAM);

	// Match
	bool try_gpu = true;
	float match_conf = 0.3f;
	vector<detail::MatchesInfo> pairwise_matches;
	detail::BestOf2NearestMatcher matcher(try_gpu, match_conf);

	// Check Panomara
	vector<int> indices;
	float conf_thresh = 1.f;

	// Homography Estimation
	detail::HomographyBasedEstimator estimator;
	vector<detail::CameraParams> cameras;
	Mat R;

	// Adjuster
	Ptr<detail::BundleAdjusterBase> adjuster;
	adjuster = new detail::BundleAdjusterReproj();
	Mat_<uchar> refine_mask;
	string ba_refine_mask = "xxxxx";

	// Find median focal length
	// Find median focal length
	vector<double> focals;
	float warped_image_scale;

	// Wave Correction
	vector<Mat> rmats;
	detail::WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;

	// Image MAsk
	vector<Point> corners(NUM_OF_CAM);
	vector<Mat> masks_warped(NUM_OF_CAM);
	vector<Size> sizes(NUM_OF_CAM);

	vector<Mat> images(NUM_OF_CAM);
	vector<Mat> images_warped_f(NUM_OF_CAM);
	vector<Mat> masks(NUM_OF_CAM);
	vector<Mat> images_warped(NUM_OF_CAM);

	// Warp images and their masks

	Ptr<WarperCreator> warper_creator;
	warper_creator = new cv::CylindricalWarper();
	//warper_creator = new cv::PlaneWarper();
	Ptr<detail::RotationWarper> warper;
	Ptr<detail::SeamFinder> seam_finder;
	seam_finder = new detail::GraphCutSeamFinder(detail::GraphCutSeamFinderBase::COST_COLOR);
	double seam_work_aspect = 1;


	Mat_<float> K;

	int expos_comp_type = detail::ExposureCompensator::GAIN_BLOCKS;
	Ptr<detail::ExposureCompensator> compensator = detail::ExposureCompensator::createDefault(expos_comp_type);



	Ptr<detail::Blender> blender;
	blender = detail::Blender::createDefault(detail::Blender::MULTI_BAND, try_gpu);
	vector<Mat> img_warped_s(NUM_OF_CAM);
	double compose_work_aspect = 1;
	Mat result;
	Mat result_mask;
	int callibration_per_n = 100;


	int n = 0;
	Mat WaistedImage;
	for (int i = 0; i<10; i++)
	{
		if (!cap1.grab())
		{
			cout << "Can not grab images." << endl;
			return -1;
		}
		else
		{
			//Mat frame2;
			cap1.retrieve(WaistedImage, 1);
			//imshow("frame1",images[0]);
		}
		if (!cap2.grab())
		{
			cout << "Can not grab images." << endl;
			return -1;
		}
		else
		{
			//Mat frame2;
			cap2.retrieve(WaistedImage, 1);
			//imshow("frame1",images[0]);
		}
		/*
		if (!cap3.grab())
		{
		cout << "Can not grab images." << endl;
		return -1;
		}
		else
		{
		//Mat frame3;
		cap3.retrieve(WaistedImage,1);
		//imshow("frame2",images[1]);
		}
		*/
		WaistedImage.release();
	}

	while (true)
	{

		int64 t = getTickCount();

		if (!cap1.grab())
		{
			cout << "Can not grab images." << endl;
			return -1;
		}
		else
		{
			//Mat frame2;
			cap1.retrieve(images[0], 1);
			imshow("frame1", images[0]);
		}

		if (!cap2.grab())
		{
			cout << "Can not grab images." << endl;
			return -1;
		}
		else
		{
			//Mat frame2;
			cap2.retrieve(images[1], 1);
			imshow("frame2", images[1]);
		}
		/*
		if (!cap3.grab())
		{
		cout << "Can not grab images." << endl;
		return -1;
		}
		else
		{
		//Mat frame3;
		cap3.retrieve(images[2],1);
		imshow("frame3",images[2]);
		}
		*/

		//if (n==0||n>callibration_per_n)
		if (n == 0)
		{
			//finder = new detail::SurfFeaturesFinder();
			//features = new vector<detail::ImageFeatures>(NUM_OF_CAM);
			//vector<detail::ImageFeatures> features(NUM_OF_CAM);

			for (int i = 0; i<NUM_OF_CAM; i++)
			{
				(*finder)(images[i], features[i]);
				features[i].img_idx = i;
			}
			finder->collectGarbage();

			matcher = new detail::BestOf2NearestMatcher(try_gpu, match_conf);
			//pairwise_matches = new vector<detail::MatchesInfo>();
			//vector<detail::MatchesInfo> pairwise_matches;
			matcher(features, pairwise_matches);

			//estimator = new detail::HomographyBasedEstimator();
			//cameras = new vector<detail::CameraParams>();
			//vector<detail::CameraParams> cameras;
			estimator(features, pairwise_matches, cameras);
			matcher.collectGarbage();


			for (size_t i = 0; i < cameras.size(); ++i)
			{
				cameras[i].R.convertTo(R, CV_32F);
				cameras[i].R = R;
			}

			//adjuster = new Ptr<detail::BundleAdjusterBase>();
			//Ptr<detail::BundleAdjusterBase> adjuster;
			adjuster->setConfThresh(conf_thresh);

			refine_mask = Mat::zeros(3, 3, CV_8U);
			if (ba_refine_mask[0] == 'x') refine_mask(0, 0) = 1;
			if (ba_refine_mask[1] == 'x') refine_mask(0, 1) = 1;
			if (ba_refine_mask[2] == 'x') refine_mask(0, 2) = 1;
			if (ba_refine_mask[3] == 'x') refine_mask(1, 1) = 1;
			if (ba_refine_mask[4] == 'x') refine_mask(1, 2) = 1;
			adjuster->setRefinementMask(refine_mask);
			(*adjuster)(features, pairwise_matches, cameras);
			//vector<double> focals;
			for (size_t i = 0; i < cameras.size(); ++i)
			{
				focals.push_back(cameras[i].focal);
			}

			sort(focals.begin(), focals.end());

			if (focals.size() % 2 == 1)
				warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
			else
				warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;
			focals.clear();
			//vector<Mat> rmats;
			for (size_t i = 0; i < cameras.size(); ++i)
				rmats.push_back(cameras[i].R);

			waveCorrect(rmats, wave_correct);
			for (size_t i = 0; i < cameras.size(); ++i)
				cameras[i].R = rmats[i];
			rmats.clear();

			for (int i = 0; i < NUM_OF_CAM; ++i)
			{
				masks[i].create(images[i].size(), CV_8U);
				masks[i].setTo(Scalar::all(255));
			}

			// Warp images and their masks


			//warper_creator = new cv::CylindricalWarper();
			warper = warper_creator->create(static_cast<float>(warped_image_scale));
			n = 1;
		}


		for (int i = 0; i < NUM_OF_CAM; ++i)
		{

			cameras[i].K().convertTo(K, CV_32F);
			//float swa = (float)seam_work_aspect;
			//K(0,0) *= swa; K(0,2) *= swa;
			//K(1,1) *= swa; K(1,2) *= swa;
			//K(0,0) = 1.0; K(0,2) = 1.0;
			//K(1,1) = 1.0; K(1,2) = 1.0;
			corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
			sizes[i] = images_warped[i].size();

			warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);


		}

		// Release unused memory


		for (int i = 0; i < NUM_OF_CAM; ++i)
			images_warped[i].convertTo(images_warped_f[i], CV_32F);

		//compensator->feed(corners, images_warped, masks_warped);



		seam_finder->find(images_warped_f, corners, masks_warped);
		//imshow("Pano",masks_warped);
		// Release unsed memory

		//images_warped.clear();

		for (int i = 0; i<NUM_OF_CAM; i++)
			images_warped[i].convertTo(img_warped_s[i], CV_16S);

		// images_warped.clear();

		blender->prepare(corners, sizes);
		for (int i = 0; i<NUM_OF_CAM; i++)
			blender->feed(img_warped_s[i], masks_warped[i], corners[i]);

		// img_warped_s.clear();
		// masks_warped.clear();
		// corners.clear();

		blender->blend(result, result_mask);

		result.convertTo(pano_fin, CV_8U);
		//images[0].convertTo(pano_fin,CV_8U);
		//corners.clear();
		imshow("Pano", pano_fin);

		n++;

		/*
		*/
		cout << "Time Elapse: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
		if (waitKey(30) >= 0)
			break;

	}
	destroyAllWindows();
	return 0;


}
