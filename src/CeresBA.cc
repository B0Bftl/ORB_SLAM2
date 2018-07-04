#include <ceres/ceres.h>
#include "ceres/rotation.h"
#include "CeresBA.h"
#include "gflags/gflags.h"

#include "snavely_reprojection_error_stereo.h"

namespace ORB_SLAM2 {


	DEFINE_string(trust_region_strategy, "dogleg",
	"Options are: levenberg_marquardt, dogleg.");
	DEFINE_string(dogleg, "traditional_dogleg", "Options are: traditional_dogleg,"
	"subspace_dogleg.");
	DEFINE_bool(inner_iterations, false, "Use inner iterations to non-linearly "
	"refine each successful trust region step.");
	DEFINE_string(blocks_for_inner_iterations, "automatic", "Options are: "
	"automatic, cameras, points, cameras,points, points,cameras");
	DEFINE_string(linear_solver, "sparse_schur", "Options are: "
	"sparse_schur, dense_schur, iterative_schur, sparse_normal_cholesky, "
	"dense_qr, dense_normal_cholesky and cgnr.");
	DEFINE_string(preconditioner, "jacobi", "Options are: "
	"identity, jacobi, schur_jacobi, cluster_jacobi, "
	"cluster_tridiagonal.");
	DEFINE_string(sparse_linear_algebra_library, "suite_sparse",
	"Options are: suite_sparse and cx_sparse.");
	DEFINE_string(dense_linear_algebra_library, "eigen",
	"Options are: eigen and lapack.");
	DEFINE_string(ordering, "automatic", "Options are: automatic, user.");
	DEFINE_bool(use_quaternions, false, "If true, uses quaternions to represent "
	"rotations. If false, angle axis is used.");
	DEFINE_bool(use_local_parameterization, false, "For quaternions, use a local "
	"parameterization.");
	DEFINE_bool(robustify, true, "Use a robust loss function.");
	DEFINE_double(eta, 1e-2, "Default value for eta. Eta determines the "
	"accuracy of each linear solve of the truncated newton step. "
	"Changing this parameter can affect solve performance.");
	DEFINE_int32(num_threads, 1, "Number of threads.");
	DEFINE_int32(num_iterations, 5, "Number of iterations.");
	DEFINE_double(max_solver_time, 1e32, "Maximum solve time in seconds.");
	DEFINE_bool(nonmonotonic_steps, false, "Trust region algorithm can use"
	" nonmonotic steps.");
	DEFINE_double(rotation_sigma, 0.0, "Standard deviation of camera rotation "
	"perturbation.");
	DEFINE_double(translation_sigma, 0.0, "Standard deviation of the camera "
	"translation perturbation.");
	DEFINE_double(point_sigma, 0.0, "Standard deviation of the point "
	"perturbation.");
	DEFINE_int32(random_seed, 38401, "Random seed used to set the state "
	"of the pseudo random number generator used to generate "
	"the pertubations.");
	DEFINE_string(solver_log, "", "File to record the solver execution to.");
	DEFINE_bool(line_search, false, "Use a line search instead of trust region "
	"algorithm.");

	void CeresBA::localBundleAdjustmentCeres(list<MapPoint *>& lLocalMapPoints,
	                                         list<KeyFrame *>& lLocalKeyFrames,
	                                         list<KeyFrame *>& lFixedKeyFrames) {

		ceres::Problem problem;
		// Map Point != Observation. Observation is one MapPoint seen by a KeyFrame
		// This means, one MapPoint can (and will) be part of many observations from different KeyFrames
		// In LocalBA, only regard KeyFrames close to the current Frame (via Covisibility Graph)
		for(list<MapPoint*>::iterator itMapPoints=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); itMapPoints!=lend; itMapPoints++)
		{
			// get current MapPoint
			MapPoint* pMapPoint = *itMapPoints;
			// we need the current 3D Position of our MapPoint

			if (pMapPoint->isBad())
				continue;

			cv::Mat mpPosition =  pMapPoint->GetWorldPos();

			// fetch all KeyFrames Observing this MapPoint
			const map<KeyFrame*,size_t> observations = pMapPoint->GetObservations();

			for(map<KeyFrame*,size_t>::const_iterator itKeyFrame=observations.begin(), mend=observations.end(); itKeyFrame!=mend; itKeyFrame++)
			{
				KeyFrame* pKeyFrame = itKeyFrame->first;

				if(pKeyFrame->isBad())
					continue;



				// We need the Translation and Rotation of the KeyFrame
				const cv::Mat kfPose = pKeyFrame->GetPose();

				ceres::CostFunction* cost_function;

				// We need the 2D Coordinates of our current MapPoint in the KeyFrame
				// Since we use a RGB-D Camera as well, out observation actually consists of 3 Coordinates
				const cv::KeyPoint &keyPointUn = pKeyFrame->mvKeysUn[itKeyFrame->second];
				const float keyPoint_ur = pKeyFrame->mvuRight[itKeyFrame->second];
				double baseline = pKeyFrame->mb;

				cost_function = ORB_SLAM2::SnavelyReprojectionErrorStereo::Create(keyPointUn.pt.x,
						                                                          keyPointUn.pt.y,
						                                                          keyPoint_ur,
						                                                          baseline);

				// use Huber's loss function.
				ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

				//std::unique_ptr<double[]> cameraPose(new double[9]);
				//std::unique_ptr<double[]> pointPosition(new double[3]);

				//double* pointPosition = new double[3];
				//double* pointPosition = new double[3];
				/*
				std::cout << "cv::Mat Pose: " << kfPose << std::endl;

				std::cout << "cameraPose: "
				    << pKeyFrame->cameraPose[0] << std::endl
					<< pKeyFrame->cameraPose[1] << std::endl
					<< pKeyFrame->cameraPose[2] << std::endl
					<< pKeyFrame->cameraPose[3] << std::endl
                    << pKeyFrame->cameraPose[4] << std::endl
                    << pKeyFrame->cameraPose[5] << std::endl
					<< pKeyFrame->cameraPose[6] << std::endl
                    << pKeyFrame->cameraPose[7] << std::endl
                    << pKeyFrame->cameraPose[8] << std::endl;

				std::cout << "cv::Mat Point: " << mpPosition << std::endl;

				std::cout << "mapPoint: "
				          << pMapPoint->pointPosition[0] << std::endl
				          << pMapPoint->pointPosition[1] << std::endl
				          << pMapPoint->pointPosition[2] << std::endl;
				*/

				problem.AddResidualBlock(cost_function, loss_function, pKeyFrame->cameraPose, pMapPoint->pointPosition);


				/*
				 * About the reprojectoin error:
				 * With our Depth Information, we do not have an image Plane anymore, but a 3D frame.
				 * So the reprojection Error is the euclidean distance between the measured Point and
				 * its' 3D coordinates, given we have a valid depth measurement.
				 * If this measurement is not given(because the point is too far away, or the sensor
				 * did not identify the depth), we use the normal reprojection.
				 *
				 * So we actually do not need to reproject the error at all with valid depth measurement
				 *
				 * Just to mention: The correct calibration of the depth sensor seems to be extremely
				 * important
				 *
				 */



			}

		}


		ceres::Solver::Options options;
		options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
		options.max_num_iterations = 100;
		options.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
		options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
		options.num_threads = 1;
		options.dense_linear_algebra_library_type = ceres::DenseLinearAlgebraLibraryType::EIGEN;
		options.gradient_tolerance = 1e-16;
		options.function_tolerance = 1e-16;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		std::cout << summary.FullReport() << "\n";
		std::cout << "done" << std::endl;


		for(list<MapPoint*>::iterator itMapPoints=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); itMapPoints!=lend; itMapPoints++)
		{
			MapPoint* pMapPoint = *itMapPoints;
			std::cout << "mapPoint: "
			          << pMapPoint->pointPosition[0] << std::endl
			          << pMapPoint->pointPosition[1] << std::endl
			          << pMapPoint->pointPosition[2] << std::endl;
			cv::Mat newPos = cv::Mat(3, 1, CV_32FC1, &(pMapPoint->pointPosition));
			pMapPoint->SetWorldPos(newPos);
			pMapPoint->UpdateNormalAndDepth();

		}


		for(list<KeyFrame*>::iterator itKeyFrames=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); itKeyFrames!=lend; itKeyFrames++)
		{
			KeyFrame* pKeyFrame = *itKeyFrames;
			std::cout << "Size: " << pKeyFrame->GetPose().size << std::endl;
			std::cout << "cameraPose: "
			          << pKeyFrame->cameraPose[0] << std::endl
			          << pKeyFrame->cameraPose[1] << std::endl
			          << pKeyFrame->cameraPose[2] << std::endl
			          << pKeyFrame->cameraPose[3] << std::endl
			          << pKeyFrame->cameraPose[4] << std::endl
			          << pKeyFrame->cameraPose[5] << std::endl
			          << pKeyFrame->cameraPose[6] << std::endl
			          << pKeyFrame->cameraPose[7] << std::endl
			          << pKeyFrame->cameraPose[8] << std::endl;

			cv::Mat newPos = cv::Mat(9, 1, CV_32FC1, &(pKeyFrame->cameraPose));
			pKeyFrame->SetPose(newPos);
		}


	}

}
