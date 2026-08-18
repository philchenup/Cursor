#include "AisRlBodyBinding.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <vector>

#include <PrsMgr_PresentableObject.hxx>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <gp.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <NCollection_Mat4.hxx>
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

bool isFinitePnt(const gp_Pnt& p)
{
	return std::isfinite(p.X()) && std::isfinite(p.Y()) && std::isfinite(p.Z());
}

} // namespace

bool isValidGpTrsf(const gp_Trsf& trsf)
{
	const double s = static_cast<double>(trsf.ScaleFactor());
	if (!std::isfinite(s) || std::fabs(s) <= static_cast<double>(gp::Resolution()))
		return false;

	switch (trsf.Form())
	{
	case gp_Identity:
	case gp_Rotation:
	case gp_Translation:
	case gp_PntMirror:
	case gp_Ax1Mirror:
	case gp_Ax2Mirror:
	case gp_Scale:
	case gp_CompoundTrsf:
	case gp_Other:
		break;
	default:
		return false;
	}

	const gp_Pnt o = gp_Pnt(0.0, 0.0, 0.0).Transformed(trsf);
	return isFinitePnt(o);
}

rl::math::Transform gpTrsfToRl(const gp_Trsf& trsf)
{
	rl::math::Transform T;
	T.setIdentity();
	if (!isValidGpTrsf(trsf))
		return T;

	// GetMat4：0-based 4×4，与 Value(1-based 3×4) 同一套系数。
	// 左上 3×3 = 旋转（含 scale），第 4 列 = 平移，末行 [0,0,0,1]。
	NCollection_Mat4<Standard_Real> m;
	trsf.GetMat4(m);

	T.linear()(0, 0) = static_cast<rl::math::Real>(m.GetValue(0, 0));
	T.linear()(0, 1) = static_cast<rl::math::Real>(m.GetValue(0, 1));
	T.linear()(0, 2) = static_cast<rl::math::Real>(m.GetValue(0, 2));
	T.linear()(1, 0) = static_cast<rl::math::Real>(m.GetValue(1, 0));
	T.linear()(1, 1) = static_cast<rl::math::Real>(m.GetValue(1, 1));
	T.linear()(1, 2) = static_cast<rl::math::Real>(m.GetValue(1, 2));
	T.linear()(2, 0) = static_cast<rl::math::Real>(m.GetValue(2, 0));
	T.linear()(2, 1) = static_cast<rl::math::Real>(m.GetValue(2, 1));
	T.linear()(2, 2) = static_cast<rl::math::Real>(m.GetValue(2, 2));
	T.translation() = rl::math::Vector3(
		static_cast<rl::math::Real>(m.GetValue(0, 3)),
		static_cast<rl::math::Real>(m.GetValue(1, 3)),
		static_cast<rl::math::Real>(m.GetValue(2, 3)));
	return T;
}

rl::math::Transform syncAisPoseToRlBody(const AIS_Shape* ais, rl::sg::Body* body)
{
	rl::math::Transform t;
	t.setIdentity();
	if (ais == nullptr || body == nullptr)
		return t;

	const gp_Trsf trsf = GetAisShapeWorldTrsf(ais);
	if (!isValidGpTrsf(trsf))
		return t;

	t = gpTrsfToRl(trsf);
	body->setFrame(t);
	return t;
}

gp_Trsf GetAisShapeWorldTrsf(const AIS_Shape* ais)
{
	return copyAisWorldTrsf(ais);
}

rl::math::Transform GetAisShapeWorldRl(const AIS_Shape* ais)
{
	return gpTrsfToRl(GetAisShapeWorldTrsf(ais));
}

const char* occAisBodyNamePrefix()
{
	return "occ_ais_body_";
}

bool isOccAisBoundBody(const rl::sg::Body* body)
{
	if (body == nullptr)
		return false;
	const std::string& name = body->getName();
	const char* prefix = occAisBodyNamePrefix();
	return name.rfind(prefix, 0) == 0;
}

namespace {

bool sameAis(const Handle(AIS_Shape)& handle, const AIS_Shape* ais)
{
	return !handle.IsNull() && handle.get() == ais;
}

} // namespace

AisRlPair* findAisRlPair(std::vector<AisRlPair>& pairs, const AIS_Shape* ais)
{
	if (ais == nullptr)
		return nullptr;
	const auto it = std::find_if(pairs.begin(), pairs.end(),
		[ais](const AisRlPair& e) { return sameAis(e.ais, ais); });
	if (it == pairs.end())
		return nullptr;
	return &(*it);
}

const AisRlPair* findAisRlPair(const std::vector<AisRlPair>& pairs, const AIS_Shape* ais)
{
	if (ais == nullptr)
		return nullptr;
	const auto it = std::find_if(pairs.begin(), pairs.end(),
		[ais](const AisRlPair& e) { return sameAis(e.ais, ais); });
	if (it == pairs.end())
		return nullptr;
	return &(*it);
}

rl::sg::Body* findRlBodyForAis(const std::vector<AisRlPair>& pairs, const AIS_Shape* ais)
{
	const AisRlPair* pair = findAisRlPair(pairs, ais);
	if (pair == nullptr)
		return nullptr;
	return pair->body;
}

void eraseAisRlPair(std::vector<AisRlPair>& pairs, const AIS_Shape* ais)
{
	if (ais == nullptr)
		return;
	const auto it = std::find_if(pairs.begin(), pairs.end(),
		[ais](const AisRlPair& e) { return sameAis(e.ais, ais); });
	if (it == pairs.end())
		return;
	unbindAisShapeFromRlBody(it->body);
	pairs.erase(it);
}

gp_Trsf copyAisLocalTrsf(const AIS_Shape* ais)
{
	gp_Trsf trsf;
	if (ais == nullptr)
		return trsf;
	trsf = ais->LocalTransformation();
	return trsf;
}

gp_Trsf copyAisWorldTrsf(const AIS_Shape* ais)
{
	gp_Trsf world;
	if (ais == nullptr)
		return world;
	world = ais->LocalTransformation();
	Handle(PrsMgr_PresentableObject) parent = ais->Parent();
	while (!parent.IsNull())
	{
		gp_Trsf parentLocal = parent->LocalTransformation();
		world.PreMultiply(parentLocal);
		parent = parent->Parent();
	}
	return world;
}

rl::math::Vector3 aisWorldTranslation(const AIS_Shape* ais)
{
	if (ais == nullptr)
		return rl::math::Vector3::Zero();
	const gp_Trsf trsf = copyAisWorldTrsf(ais);
	if (!isValidGpTrsf(trsf))
		return rl::math::Vector3::Zero();
	const gp_Pnt o = gp_Pnt(0.0, 0.0, 0.0).Transformed(trsf);
	return rl::math::Vector3(
		static_cast<rl::math::Real>(o.X()),
		static_cast<rl::math::Real>(o.Y()),
		static_cast<rl::math::Real>(o.Z()));
}

rl::math::Vector3 rlBodyTranslation(const rl::sg::Body* body)
{
	if (body == nullptr)
		return rl::math::Vector3::Zero();
	const rl::math::Transform T = body->getFrame();
	return T.translation();
}

rl::sg::Body* findRlBodyByName(rl::sg::Model* sgModel, const std::string& name)
{
	if (sgModel == nullptr)
		return nullptr;
	for (std::size_t i = 0; i < sgModel->getNumBodies(); ++i)
	{
		rl::sg::Body* body = sgModel->getBody(i);
		if (body != nullptr && body->getName() == name)
			return body;
	}
	return nullptr;
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

	static std::uint64_t s_nextId = 0;
	std::ostringstream name;
	name << occAisBodyNamePrefix() << ++s_nextId;
	body->setName(name.str());

	body->create(vrml);
	vrml->unref();

	syncAisPoseToRlBody(ais, body);
	return body;
}

void unbindAisShapeFromRlBody(rl::sg::Body*& body)
{
	if (body == nullptr)
		return;
	delete body;
	body = nullptr;
}

void unbindAisShapesFromRlModel(rl::sg::Model* sgModel)
{
	if (sgModel == nullptr)
		return;

	for (std::size_t i = sgModel->getNumBodies(); i > 0; )
	{
		--i;
		rl::sg::Body* body = sgModel->getBody(i);
		if (isOccAisBoundBody(body))
			delete body;
	}
}
