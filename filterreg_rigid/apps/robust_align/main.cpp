#include "io/pcd_io.h"
#include "registration/align.h"
#include "registration/preprocess.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace {

using small_reg::AlignResult;
using small_reg::AlignSetting;
using small_reg::Method;
using small_reg::PointCloud;

void AddPlane(PointCloud& cloud,
              const Eigen::Vector3d& origin,
              const Eigen::Vector3d& u,
              const Eigen::Vector3d& v,
              int nu,
              int nv,
              std::mt19937& rng,
              double noise) {
	std::normal_distribution<double> n(0.0, noise);
	for (int i = 0; i < nu; ++i) {
		for (int j = 0; j < nv; ++j) {
			const double su = (nu == 1) ? 0.0 : static_cast<double>(i) / (nu - 1);
			const double sv = (nv == 1) ? 0.0 : static_cast<double>(j) / (nv - 1);
			Eigen::Vector3d p = origin + su * u + sv * v;
			p.x() += n(rng);
			p.y() += n(rng);
			p.z() += n(rng);
			cloud.points.push_back(p);
		}
	}
}

// Indoor-like degenerate scene: floor + L-walls + a parallel tabletop.
// The table/floor pair is what makes a too-large GICP max_corr_dist fail.
PointCloud MakePlanarScene(std::mt19937& rng, double noise) {
	PointCloud cloud;
	AddPlane(cloud, {-1.5, -1.5, 0.0}, {3.0, 0, 0}, {0, 3.0, 0}, 40, 40, rng, noise);       // floor
	AddPlane(cloud, {-1.5, -1.5, 0.0}, {0, 3.0, 0}, {0, 0, 1.6}, 32, 18, rng, noise);        // wall y
	AddPlane(cloud, {-1.5, -1.5, 0.0}, {3.0, 0, 0}, {0, 0, 1.6}, 32, 18, rng, noise);        // wall x
	AddPlane(cloud, {-0.35, -0.35, 0.75}, {0.7, 0, 0}, {0, 0.7, 0}, 16, 16, rng, noise);     // table
	return cloud;
}

PointCloud TransformCloud(const PointCloud& in, const Eigen::Isometry3d& T) {
	PointCloud out;
	out.points.reserve(in.size());
	for (const auto& p : in.points) out.points.push_back(T * p);
	return out;
}

Eigen::Isometry3d MakeGt() {
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	T.linear() = Eigen::AngleAxisd(6.0 * 3.14159265358979323846 / 180.0,
	                               Eigen::Vector3d::UnitZ()).toRotationMatrix();
	T.translation() = Eigen::Vector3d(0.03, -0.02, 0.07);  // 7 cm vertical gap
	return T;
}

void PrintResult(const char* name, const AlignResult& r, const Eigen::Isometry3d& gt, double rmse) {
	const Eigen::Isometry3d err = r.T.inverse() * gt;
	const double trans = err.translation().norm();
	const double rot = Eigen::AngleAxisd(err.linear()).angle() * 180.0 / 3.14159265358979323846;
	std::cout << name
	          << " success=" << (r.success ? "yes" : "no")
	          << " inliers=" << r.num_inliers
	          << " overlap=" << r.overlap
	          << " max_d=" << r.final_max_dist
	          << " trans_err=" << trans << " m"
	          << " rot_err=" << rot << " deg"
	          << " rmse=" << rmse << " m"
	          << " iters=" << r.iterations
	          << "\n";
}

bool ParseMethod(const std::string& s, Method* m) {
	if (s == "icp" || s == "p2p") {
		*m = Method::ICP;
		return true;
	}
	if (s == "plane" || s == "p2l" || s == "plane_icp") {
		*m = Method::PlaneICP;
		return true;
	}
	if (s == "gicp") {
		*m = Method::GICP;
		return true;
	}
	return false;
}

PointCloud FromPcdXYZ(const std::vector<poser::PcdPointXYZ>& pts) {
	PointCloud c;
	c.points.reserve(pts.size());
	for (const auto& p : pts) c.points.emplace_back(p.x, p.y, p.z);
	return c;
}

PointCloud FromPcdXYZN(const std::vector<poser::PcdPointXYZNormal>& pts) {
	PointCloud c;
	c.points.reserve(pts.size());
	c.normals.reserve(pts.size());
	for (const auto& p : pts) {
		c.points.emplace_back(p.x, p.y, p.z);
		c.normals.emplace_back(p.normal_x, p.normal_y, p.normal_z);
	}
	return c;
}

bool LoadCloud(const std::string& path, PointCloud* cloud) {
	std::vector<poser::PcdPointXYZNormal> with_n;
	if (poser::LoadPcdXYZNormal(path, with_n) && !with_n.empty()) {
		*cloud = FromPcdXYZN(with_n);
		return true;
	}
	std::vector<poser::PcdPointXYZ> xyz;
	if (poser::LoadPcdXYZ(path, xyz) && !xyz.empty()) {
		*cloud = FromPcdXYZ(xyz);
		return true;
	}
	return false;
}

void SavePLY(const PointCloud& cloud, const Eigen::Isometry3d& T, const std::string& path) {
	std::ofstream ofs(path);
	ofs << "ply\nformat ascii 1.0\n";
	ofs << "element vertex " << cloud.size() << "\n";
	ofs << "property float x\nproperty float y\nproperty float z\nend_header\n";
	for (const auto& p : cloud.points) {
		const Eigen::Vector3d q = T * p;
		ofs << q.x() << " " << q.y() << " " << q.z() << "\n";
	}
}

int RunDemo() {
	std::mt19937 rng(7);
	const Eigen::Isometry3d gt = MakeGt();
	const PointCloud target = MakePlanarScene(rng, 0.004);
	const PointCloud source = TransformCloud(MakePlanarScene(rng, 0.004), gt.inverse());

	std::cout << "Synthetic planar scene: |target|=" << target.size()
	          << " |source|=" << source.size() << "\n";
	std::cout << "GT translation " << gt.translation().transpose()
	          << " m, rotation 6 deg about Z (7 cm vertical gap + table at z=0.75).\n\n";

	auto run = [&](const char* name, const PointCloud& tgt, const PointCloud& src,
	               AlignSetting s, bool multi) {
		s.verbose = false;
		s.multiscale = multi;
		s.voxel = 0.04;
		const AlignResult r = small_reg::Align(tgt, src, Eigen::Isometry3d::Identity(), s);
		const double rmse = small_reg::CloudRMSE(tgt, src, r.T, 0.15);
		PrintResult(name, r, gt, rmse);
		return r;
	};

	std::cout << "== Floor + L-walls + table (rotation is observable) ==\n";
	AlignSetting p2p;
	p2p.method = Method::ICP;
	p2p.max_corr_dist = 0.5;
	p2p.adaptive_max_dist = false;
	p2p.multiscale = false;
	p2p.normal_check = false;
	p2p.robust_kernel = false;
	const AlignResult r_p2p = run("P2P ICP  max_d=0.50 no-robust", target, source, p2p, false);

	AlignSetting p2l;
	p2l.method = Method::PlaneICP;
	p2l.max_corr_dist = 0.5;
	p2l.adaptive_max_dist = false;
	p2l.normal_check = true;
	p2l.robust_kernel = false;
	const AlignResult r_p2l = run("P2L ICP  max_d=0.50 no-robust", target, source, p2l, false);

	AlignSetting gicp_tight;
	gicp_tight.method = Method::GICP;
	gicp_tight.max_corr_dist = 0.18;
	gicp_tight.adaptive_max_dist = false;
	gicp_tight.normal_check = true;
	gicp_tight.robust_kernel = true;
	gicp_tight.multiscale = false;
	const AlignResult r_tight = run("GICP     max_d=0.18 +Cauchy+ncheck", target, source, gicp_tight, false);

	AlignSetting gicp_too_small;
	gicp_too_small.method = Method::GICP;
	gicp_too_small.max_corr_dist = 0.03;  // smaller than the 7 cm gap → no correspondences
	gicp_too_small.adaptive_max_dist = false;
	gicp_too_small.normal_check = false;
	gicp_too_small.robust_kernel = false;
	gicp_too_small.multiscale = false;
	const AlignResult r_small = run("GICP     max_d=0.03 (too small)", target, source, gicp_too_small, false);

	AlignSetting gicp_adapt;
	gicp_adapt.method = Method::GICP;
	gicp_adapt.max_corr_dist = 1.2;
	gicp_adapt.adaptive_max_dist = true;
	gicp_adapt.normal_check = true;
	gicp_adapt.robust_kernel = true;
	gicp_adapt.multiscale = true;
	const AlignResult r_adapt = run("GICP     max_d=1.20 adaptive+multi", target, source, gicp_adapt, true);

	std::mt19937 rng2(11);
	PointCloud floor_table;
	AddPlane(floor_table, {-1.2, -1.2, 0.0}, {2.4, 0, 0}, {0, 2.4, 0}, 48, 48, rng2, 0.004);
	AddPlane(floor_table, {-0.7, -0.7, 0.55}, {1.4, 0, 0}, {0, 1.4, 0}, 40, 40, rng2, 0.004);
	const PointCloud ft_src = TransformCloud(floor_table, gt.inverse());
	std::cout << "\n== Floor + large parallel table at z=0.55 (GICP d_max trap) ==\n";
	std::cout << "|cloud|=" << floor_table.size() << "\n";

	AlignSetting gicp_wide;
	gicp_wide.method = Method::GICP;
	gicp_wide.max_corr_dist = 1.0;
	gicp_wide.adaptive_max_dist = false;
	gicp_wide.normal_check = false;
	gicp_wide.robust_kernel = false;
	gicp_wide.multiscale = false;
	const AlignResult r_wide = run("GICP     max_d=1.00 no-robust", floor_table, ft_src, gicp_wide, false);

	AlignSetting gicp_rescue;
	gicp_rescue.method = Method::GICP;
	gicp_rescue.max_corr_dist = 1.0;
	gicp_rescue.adaptive_max_dist = true;
	gicp_rescue.normal_check = true;
	gicp_rescue.robust_kernel = true;
	gicp_rescue.multiscale = true;
	const AlignResult r_rescue = run("GICP     max_d=1.00 adaptive+multi", floor_table, ft_src, gicp_rescue, true);
	std::cout << "(Two parallel planes leave yaw weakly observed; residual in-plane error is expected.)\n";

	const Eigen::Isometry3d err = r_adapt.T.inverse() * gt;
	const double trans = err.translation().norm();
	const Eigen::Isometry3d err_p2p = r_p2p.T.inverse() * gt;
	const bool p2p_worse = err_p2p.translation().norm() > 0.02;
	const bool pass = r_adapt.success && trans < 0.03 && p2p_worse;
	std::cout << "\nSelf-check: P2P residual " << err_p2p.translation().norm()
	          << " m (gap expected); adaptive GICP " << trans << " m"
	          << " => " << (pass ? "PASS" : "FAIL") << "\n";
	(void)r_p2l;
	(void)r_tight;
	(void)r_small;
	(void)r_wide;
	(void)r_rescue;
	return pass ? 0 : 1;
}

void PrintUsage(const char* argv0) {
	std::cerr
		<< "Usage:\n"
		<< "  " << argv0 << " --demo\n"
		<< "  " << argv0 << " <source.pcd> <target.pcd> [options]\n"
		<< "Options:\n"
		<< "  --method icp|plane|gicp   (default gicp)\n"
		<< "  --voxel <m>               (default 0.05)\n"
		<< "  --max-dist <m>            (default 0.5)\n"
		<< "  --no-adaptive             disable shrinking correspondence gate\n"
		<< "  --no-robust               disable Cauchy kernel\n"
		<< "  --no-multiscale           single-resolution only\n"
		<< "  --no-normal-check         do not reject disagreeing normals\n"
		<< "  --verbose\n";
}

}  // namespace

int main(int argc, char** argv) {
	if (argc < 2) {
		PrintUsage(argv[0]);
		return 2;
	}
	const std::string a1 = argv[1];
	if (a1 == "--demo" || a1 == "--selftest") return RunDemo();
	if (a1 == "-h" || a1 == "--help") {
		PrintUsage(argv[0]);
		return 0;
	}
	if (argc < 3) {
		PrintUsage(argv[0]);
		return 2;
	}

	PointCloud source, target;
	if (!LoadCloud(argv[1], &source)) {
		std::cerr << "Failed to load source " << argv[1] << "\n";
		return 1;
	}
	if (!LoadCloud(argv[2], &target)) {
		std::cerr << "Failed to load target " << argv[2] << "\n";
		return 1;
	}

	AlignSetting s;
	for (int i = 3; i < argc; ++i) {
		const std::string a = argv[i];
		auto need = [&](double* dst) {
			if (i + 1 >= argc) return false;
			*dst = std::atof(argv[++i]);
			return true;
		};
		if (a == "--method" && i + 1 < argc) {
			if (!ParseMethod(argv[++i], &s.method)) {
				std::cerr << "Unknown method\n";
				return 2;
			}
		} else if (a == "--voxel") {
			if (!need(&s.voxel)) return 2;
		} else if (a == "--max-dist") {
			if (!need(&s.max_corr_dist)) return 2;
		} else if (a == "--no-adaptive") {
			s.adaptive_max_dist = false;
		} else if (a == "--no-robust") {
			s.robust_kernel = false;
		} else if (a == "--no-multiscale") {
			s.multiscale = false;
		} else if (a == "--no-normal-check") {
			s.normal_check = false;
		} else if (a == "--verbose") {
			s.verbose = true;
		} else {
			std::cerr << "Unknown arg " << a << "\n";
			PrintUsage(argv[0]);
			return 2;
		}
	}

	std::cout << "source=" << source.size() << " target=" << target.size()
	          << " method=" << small_reg::MethodName(s.method)
	          << " voxel=" << s.voxel
	          << " max_dist=" << s.max_corr_dist << "\n";

	const AlignResult r = small_reg::Align(target, source, Eigen::Isometry3d::Identity(), s);
	std::cout << "success=" << (r.success ? "yes" : "no")
	          << " inliers=" << r.num_inliers
	          << " overlap=" << r.overlap
	          << " fitness=" << r.fitness
	          << " final_max_dist=" << r.final_max_dist
	          << " iters=" << r.iterations
	          << " converged=" << (r.converged ? "yes" : "no") << "\n";
	std::cout << "T =\n" << r.T.matrix() << "\n";

	SavePLY(source, r.T, "aligned_source.ply");
	SavePLY(target, Eigen::Isometry3d::Identity(), "aligned_target.ply");
	std::cout << "Wrote aligned_source.ply and aligned_target.ply\n";
	const double rmse = small_reg::CloudRMSE(target, source, r.T, std::max(0.1, 3.0 * s.voxel));
	std::cout << "inlier RMSE (3*voxel) = " << rmse << " m\n";
	return r.success ? 0 : 1;
}
