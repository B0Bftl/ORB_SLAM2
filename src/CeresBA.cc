#include <ceres/ceres.h>
#include "ceres/rotation.h"
#include "CeresBA.h"
#include "gflags/gflags.h"

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



	struct SnavelyReprojectionError {
		SnavelyReprojectionError(double observed_x, double observed_y)
				: observed_x(observed_x), observed_y(observed_y) {}
		template <typename T>
		bool operator()(const T* const camera,
		                const T* const point,
		                T* residuals) const {
			// camera[0,1,2] are the angle-axis rotation.
			T p[3];
			ceres::AngleAxisRotatePoint(camera, point, p);
			// camera[3,4,5] are the translation.
			p[0] += camera[3];
			p[1] += camera[4];
			p[2] += camera[5];
			// Compute the center of distortion. The sign change comes from
			// the camera model that Noah Snavely's Bundler assumes, whereby
			// the camera coordinate system has a negative z axis.
			T xp = - p[0] / p[2];
			T yp = - p[1] / p[2];
			// Apply second and fourth order radial distortion.
			const T& l1 = camera[7];
			const T& l2 = camera[8];
			T r2 = xp*xp + yp*yp;
			T distortion = T(1.0) + r2  * (l1 + l2  * r2);
			// Compute final projected point position.
			const T& focal = camera[6];
			T predicted_x = focal * distortion * xp;
			T predicted_y = focal * distortion * yp;
			// The error is the difference between the predicted and observed position.
			residuals[0] = predicted_x - T(observed_x);
			residuals[1] = predicted_y - T(observed_y);
			return true;
		}
		// Factory to hide the construction of the CostFunction object from
		// the client code.
		static ceres::CostFunction* Create(const double observed_x,
		                                   const double observed_y) {
			return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
					new SnavelyReprojectionError(observed_x, observed_y)));
		}
		double observed_x;
		double observed_y;
	};


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
			ceres::CostFunction* cost_function;
			// we need the current 3D Position of our MapPoint
			cv::Mat positionMapPoint =  pMapPoint->GetWorldPos();

			// fetch all KeyFrames Observing this MapPoint
			const map<KeyFrame*,size_t> observations = pMapPoint->GetObservations();


			for(map<KeyFrame*,size_t>::const_iterator itKeyFrame=observations.begin(), mend=observations.end(); itKeyFrame!=mend; itKeyFrame++)
			{
				KeyFrame* pKeyFrame = itKeyFrame->first;

				if(pKeyFrame->isBad())
					continue;

				// We need the Translation and Rotation of the KeyFrame
				const cv::Mat kfTranslation = pKeyFrame->GetTranslation();
				const cv::Mat kfRotation = pKeyFrame->GetRotation();

				// We need the 2D Coordinates of our current MapPoint in the KeyFrame
				const cv::KeyPoint &keyPointUn = pKeyFrame->mvKeysUn[itKeyFrame->second];
				Eigen::Matrix<double,3,1> obs;
				const float keyPoint_ur = pKeyFrame->mvuRight[itKeyFrame->second];
				obs << keyPointUn.pt.x, keyPointUn.pt.y, keyPoint_ur;





			}


			cost_function = SnavelyReprojectionError::Create(pMapPoint->mTrackProjX, pMapPoint->mTrackProjY);

			ceres::LossFunction* loss_function  = new ceres::HuberLoss(1.0);

			pMapPoint->GetObservations();

			// Now need the KeyFrames, that see the current Map Point
			problem.AddResidualBlock(cost_function, loss_function, )

		}



	}



}

