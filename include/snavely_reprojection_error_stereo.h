// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)
//
// Templated struct implementing the camera model and residual
// computation for bundle adjustment used by Noah Snavely's Bundler
// SfM system. This is also the camera model/residual for the bundle
// adjustment problems in the BAL dataset. It is templated so that we
// can use Ceres's automatic differentiation to compute analytic
// jacobians.
//
// For details see: http://phototour.cs.washington.edu/bundler/
// and http://grail.cs.washington.edu/projects/bal/


#ifndef SNAVELY_REPROJECTION_ERROR_H
#define SNAVELY_REPROJECTION_ERROR_H

#include "ceres/rotation.h"

namespace ORB_SLAM2 {
		
// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
// the observed Points are Feature Points on the Image Plane, already undistorted
		struct SnavelyReprojectionErrorStereo {
			SnavelyReprojectionErrorStereo(double observed_lx, double observed_ly, double observerd_rx, double baseline)
					: observed_lx(observed_lx), observed_ly(observed_ly), observed_rx(observerd_rx), baseline(baseline) {}

			template <typename T>
			bool operator()(const T* const camera,
			                const T* const point,
			                T* residuals) const {

				// camera is our current KeyFrame
				// point is the MapPoint in our 3D coordinate System

				// camera[0,1,2] are the angle-axis rotation.
				T p[3];
				ceres::AngleAxisRotatePoint(camera, point, p);

				// camera[3,4,5] are the translation.
				p[0] += camera[3];
				p[1] += camera[4];
				p[2] += camera[5];

				// now we want to undistort

				// Compute the center of distortion. The sign change comes from
				// the camera model that Noah Snavely's Bundler assumes, whereby
				// the camera coordinate system has a negative z axis.
				const T left_xp = - p[0] / p[2];
				const T left_yp = - p[1] / p[2];
				// the stereo coordinate derived from rgb-d
				const T right_xp = - (p[0] - baseline)/ p[2];

				// xp and yp now represent our reprojected 2D Points on the image plain from our given 3D
				// feature Points. Not sure about the sign though
				// to reproject accurately we now need to apply the camera distortion to our reprojected Points.

				// Apply second and fourth order radial distortion.
				const T& l1 = camera[7];
				const T& l2 = camera[8];
				const T r2 = left_xp*left_xp + left_yp*left_yp;
				const T distortion = 1.0 + r2  * (l1 + l2  * r2);


				// Compute final projected point position.
				const T& focal = camera[6];

				/*
				 * without distortion we have the formula fx * y_1/y_3 + p
				 * This Formula is the same as we see in Strasdat et al "Double Window Optimisation for Constant
				 * Time Visual SLAM". Out 2D Feature Points identified on the image plain are already undistorted.
				 * So do we still need this distortion correction?
				 * To correctly reproject from an unknown 3D Point we still should undistort.
				 * The reprojection still suffers from distortion, only the given observations are undistorted
				 *
				 * What about our Stereo (calculated from depth) prediction. Since it is derived from the
				 * point in the left, the observation is undistorted. Most likely we need to apply the same method
				 * of undistortion as we do for the left coordinate
				 */
				const T predicted_lx = focal * distortion * left_xp;
				const T predicted_ly = focal * distortion * left_yp;
				// stereo prediction
				const T predicted_rx = focal * distortion * right_xp;

				// The error is the difference between the predicted and observed position.
				residuals[0] = predicted_lx - observed_lx;
				residuals[1] = predicted_ly - observed_ly;
				residuals[2] = predicted_rx - observed_rx;

				return true;
			}

			// Factory to hide the construction of the CostFunction object from
			// the client code.
			static ceres::CostFunction* Create(const double observed_lx,
			                                   const double observed_ly,
			                                   const double observed_rx,
			                                   const double baseline) {
				return (new ceres::AutoDiffCostFunction<SnavelyReprojectionErrorStereo, 3, 9, 3>(
						new SnavelyReprojectionErrorStereo(observed_lx, observed_ly, observed_rx, baseline)));
			}

			double observed_lx;
			double observed_ly;
			double observed_rx;
			double baseline;
		};
}  // namespace ORB_SLAM2

#endif //SNAVELY_REPROJECTION_ERROR_H
