#include "AisRlBodyBinding.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <gp_Pnt.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_XYZ.hxx>
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

} // namespace

rl::math::Transform gpTrsfToRl(const gp_Trsf& trsf)
{
	// OCCT:  p' = ScaleFactor * GetRotation * p + TranslationPart
	//        GetRotation() 来自 HVectorialPart，不含比例；Value()/VectorialPart() 含比例。
	// RL:    typedef Eigen::Transform<Real, 3, Eigen::Affine>
	//        官方写法是 linear() / translation()，不要用 T(i,j) 去填 3x3。
	// Eigen Quaternion(w,x,y,z)；OCCT 是 q.W()/X()/Y()/Z()，构造函数参数顺序相反。
	const gp_Quaternion q = trsf.GetRotation();
	const gp_XYZ& p = trsf.TranslationPart();
	const rl::math::Real s = static_cast<rl::math::Real>(trsf.ScaleFactor());

	rl::math::Transform T;
	T.setIdentity();
	T.linear() = s * rl::math::Quaternion(
			static_cast<rl::math::Real>(q.W()),
			static_cast<rl::math::Real>(q.X()),
			static_cast<rl::math::Real>(q.Y()),
			static_cast<rl::math::Real>(q.Z()))
		.normalized()
		.toRotationMatrix();
	T.translation() = rl::math::Vector3(
		static_cast<rl::math::Real>(p.X()),
		static_cast<rl::math::Real>(p.Y()),
		static_cast<rl::math::Real>(p.Z()));
	return T;
}

void syncAisPoseToRlBody(const AIS_Shape* ais, rl::sg::Body* body)
{
	if (ais == nullptr || body == nullptr)
		return;
	body->setFrame(gpTrsfToRl(ais->Transformation()));
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
