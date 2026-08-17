#include "AisRlBodyBinding.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Standard_Failure.hxx>
#include <gp_Mat.hxx>
#include <gp_Pnt.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>

namespace {

SoVRMLShape* makeVrmlMesh(const TopoDS_Shape& shape, Standard_Real deflection)
{
	Bnd_Box box;
	BRepBndLib::Add(shape, box);
	Standard_Real linDefl = deflection;
	if (linDefl <= 0.0)
	{
		if (box.IsVoid())
			linDefl = 1.0;
		else
			linDefl = 0.01 * std::sqrt(box.SquareExtent());
	}

	BRepMesh_IncrementalMesh mesher(shape, linDefl, Standard_False, 0.5, Standard_True);
	(void)mesher;

	std::vector<SbVec3f> points;
	std::vector<int32_t> indices;
	points.reserve(4096);
	indices.reserve(8192);

	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
	{
		const TopoDS_Face face = TopoDS::Face(exp.Current());
		TopLoc_Location loc;
		Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
		if (tri.IsNull() || tri->NbTriangles() < 1)
			continue;

		const gp_Trsf trsf = loc.Transformation();
		const Standard_Integer i0 = static_cast<Standard_Integer>(points.size());
		points.reserve(points.size() + static_cast<std::size_t>(tri->NbNodes()));

		for (Standard_Integer n = 1; n <= tri->NbNodes(); ++n)
		{
			const gp_Pnt p = tri->Node(n).Transformed(trsf);
			points.emplace_back(
				static_cast<float>(p.X()),
				static_cast<float>(p.Y()),
				static_cast<float>(p.Z()));
		}

		const Standard_Boolean reversed = (face.Orientation() == TopAbs_REVERSED);
		for (Standard_Integer t = 1; t <= tri->NbTriangles(); ++t)
		{
			Standard_Integer n1, n2, n3;
			tri->Triangle(t).Get(n1, n2, n3);
			if (reversed)
				std::swap(n2, n3);
			indices.push_back(i0 + n1 - 1);
			indices.push_back(i0 + n2 - 1);
			indices.push_back(i0 + n3 - 1);
			indices.push_back(-1);
		}
	}

	if (points.empty() || indices.size() < 4)
		return nullptr;

	SoVRMLCoordinate* coord = new SoVRMLCoordinate();
	coord->point.setNum(static_cast<int>(points.size()));
	SbVec3f* dst = coord->point.startEditing();
	for (std::size_t i = 0; i < points.size(); ++i)
		dst[i] = points[i];
	coord->point.finishEditing();

	SoVRMLIndexedFaceSet* faces = new SoVRMLIndexedFaceSet();
	faces->coord.setValue(coord);
	faces->ccw = TRUE;
	faces->solid = TRUE;
	faces->convex = FALSE;
	faces->coordIndex.setValues(0, static_cast<int>(indices.size()), indices.data());

	SoVRMLAppearance* app = new SoVRMLAppearance();
	app->material.setValue(new SoVRMLMaterial());

	SoVRMLShape* vrml = new SoVRMLShape();
	vrml->appearance.setValue(app);
	vrml->geometry.setValue(faces);
	return vrml;
}

constexpr double kRigidEps2 = 1e-24;

bool isFinite3(double x, double y, double z)
{
	return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

using Rot3 = ::Eigen::Matrix<rl::math::Real, 3, 3>;

bool copyGpMat(const gp_Mat& m, Rot3& R)
{
	for (int row = 0; row < 3; ++row)
	{
		for (int col = 0; col < 3; ++col)
			R(row, col) = static_cast<rl::math::Real>(m.Value(row + 1, col + 1));
	}
	return R.allFinite();
}

bool orthonormalizeRightHanded(Rot3& R)
{
	rl::math::Vector3 x = R.col(0);
	if (!x.allFinite() || x.squaredNorm() < kRigidEps2)
		return false;
	x.normalize();

	rl::math::Vector3 y = R.col(1);
	if (!y.allFinite())
		return false;
	y -= x * x.dot(y);
	if (y.squaredNorm() < kRigidEps2)
		y = x.unitOrthogonal();
	else
		y.normalize();

	rl::math::Vector3 z = x.cross(y);
	if (!z.allFinite() || z.squaredNorm() < kRigidEps2)
		return false;
	z.normalize();

	R.col(0) = x;
	R.col(1) = y;
	R.col(2) = z;
	return R.allFinite();
}

bool rotationFromQuaternion(const gp_Quaternion& q, Rot3& R)
{
	const double w = q.W();
	const double x = q.X();
	const double y = q.Y();
	const double z = q.Z();
	if (!std::isfinite(w) || !isFinite3(x, y, z))
		return false;

	const double n2 = w * w + x * x + y * y + z * z;
	if (n2 < kRigidEps2)
		return false;

	const double inv = 1.0 / std::sqrt(n2);
	const rl::math::Quaternion eq(
		static_cast<rl::math::Real>(w * inv),
		static_cast<rl::math::Real>(x * inv),
		static_cast<rl::math::Real>(y * inv),
		static_cast<rl::math::Real>(z * inv));
	R = eq.toRotationMatrix();
	return R.allFinite();
}

} // namespace

rl::math::Transform gpTrsfToRl(const gp_Trsf& trsf)
{
	// RL / Inventor / Bullet 的 setFrame 只接受刚体（SO(3)+平移）。
	// 旧实现把 VectorialPart()（旋转×比例）写进 T(i,j)：
	// 非正交 3x3、NaN，或未写齐次行时，后续分解会直接崩溃。
	rl::math::Transform T;
	T.setIdentity();

	try
	{
		const gp_XYZ p = trsf.TranslationPart();
		if (!isFinite3(p.X(), p.Y(), p.Z()))
			return T;

		T.translation() = rl::math::Vector3(
			static_cast<rl::math::Real>(p.X()),
			static_cast<rl::math::Real>(p.Y()),
			static_cast<rl::math::Real>(p.Z()));

		::Eigen::Matrix<rl::math::Real, 3, 3> R =
			::Eigen::Matrix<rl::math::Real, 3, 3>::Identity();
		const bool gotRotation =
			rotationFromQuaternion(trsf.GetRotation(), R) ||
			(copyGpMat(trsf.HVectorialPart(), R) && orthonormalizeRightHanded(R));
		if (!gotRotation)
			return T;

		T.linear() = R;
		return T;
	}
	catch (const Standard_Failure&)
	{
		T.setIdentity();
		return T;
	}
	catch (...)
	{
		T.setIdentity();
		return T;
	}
}

void syncAisPoseToRlBody(const AIS_Shape* ais, rl::sg::Body* body)
{
	if (ais == nullptr || body == nullptr)
		return;
	try
	{
		body->setFrame(gpTrsfToRl(ais->Transformation()));
	}
	catch (const Standard_Failure&)
	{
	}
	catch (...)
	{
	}
}

rl::sg::Body* bindAisShapeToRlBody(
	AIS_Shape* ais,
	rl::sg::Model* sgModel,
	Standard_Real linearDeflection)
{
	if (ais == nullptr || sgModel == nullptr)
		return nullptr;

	const TopoDS_Shape shape = ais->Shape();
	if (shape.IsNull())
		return nullptr;

	SoVRMLShape* vrml = makeVrmlMesh(shape, linearDeflection);
	if (vrml == nullptr)
		return nullptr;

	vrml->ref();
	rl::sg::Body* body = sgModel->create();
	body->setName("occ_ais_body");
	body->create(vrml);
	vrml->unref();

	syncAisPoseToRlBody(ais, body);
	return body;
}
