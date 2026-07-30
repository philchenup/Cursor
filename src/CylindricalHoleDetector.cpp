#include "CylindricalHoleDetector.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <pcl/io/pcd_io.h>

#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <gp_Vec.hxx>

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

CylindricalHoleDetector::CylindricalHoleDetector(std::string step_path,
                                                 double diameter_mm,
                                                 double tolerance_mm)
    : step_path_(std::move(step_path)),
      diameter_mm_(diameter_mm),
      tolerance_mm_(tolerance_mm),
      cloud_(new pcl::PointCloud<pcl::PointXYZ>()) {}

void CylindricalHoleDetector::setMeshDeflection(double deflection_mm) {
  mesh_deflection_mm_ = deflection_mm;
}

void CylindricalHoleDetector::setMeshAngularDeflection(double angle_rad) {
  mesh_angular_deflection_ = angle_rad;
}

void CylindricalHoleDetector::setHolesOnly(bool holes_only) {
  holes_only_ = holes_only;
}

void CylindricalHoleDetector::setAngleToleranceDeg(double angle_tol_deg) {
  angle_tol_deg_ = angle_tol_deg;
}

void CylindricalHoleDetector::setAxisDistanceTolerance(double distance_tol_mm) {
  axis_distance_tol_mm_ = distance_tol_mm;
}

bool CylindricalHoleDetector::process() {
  last_error_.clear();
  holes_.clear();
  cloud_->clear();

  if (!loadStep()) {
    return false;
  }

  TopoDS_Shape solid = extractSolid(shape_);
  if (solid.IsNull()) {
    last_error_ = "No solid found in STEP model";
    return false;
  }

  const std::vector<CylinderFace> faces = collectCylinderFaces(solid);
  holes_ = mergeCoaxial(faces);

  std::sort(holes_.begin(), holes_.end(),
            [](const HoleInfo& a, const HoleInfo& b) {
              if (a.area_mm2 != b.area_mm2) {
                return a.area_mm2 > b.area_mm2;
              }
              return a.diameter_mm < b.diameter_mm;
            });

  if (!samplePointCloud()) {
    return false;
  }
  return true;
}

bool CylindricalHoleDetector::loadStep() {
  STEPControl_Reader reader;
  const IFSelect_ReturnStatus status = reader.ReadFile(step_path_.c_str());
  if (status != IFSelect_RetDone) {
    last_error_ = "Failed to read STEP file: " + step_path_;
    return false;
  }

  reader.TransferRoots();
  shape_ = reader.OneShape();
  if (shape_.IsNull()) {
    last_error_ = "STEP file contains no transferable shape: " + step_path_;
    return false;
  }
  return true;
}

TopoDS_Shape CylindricalHoleDetector::extractSolid(
    const TopoDS_Shape& shape) const {
  if (shape.ShapeType() == TopAbs_SOLID) {
    return shape;
  }

  TopExp_Explorer explorer(shape, TopAbs_SOLID);
  if (!explorer.More()) {
    return shape;
  }

  TopoDS_Shape first = explorer.Current();
  explorer.Next();
  // Unique solid → use it for classification; otherwise keep root shape.
  if (!explorer.More()) {
    return first;
  }
  return shape;
}

std::vector<CylindricalHoleDetector::CylinderFace>
CylindricalHoleDetector::collectCylinderFaces(const TopoDS_Shape& solid) const {
  std::vector<CylinderFace> raw;

  for (TopExp_Explorer explorer(shape_, TopAbs_FACE); explorer.More();
       explorer.Next()) {
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    BRepAdaptor_Surface adaptor(face, Standard_True);
    if (adaptor.GetType() != GeomAbs_Cylinder) {
      continue;
    }

    const gp_Cylinder cylinder = adaptor.Cylinder();
    const double radius = cylinder.Radius();
    const double diameter = radius * 2.0;

    if (std::abs(diameter - diameter_mm_) > tolerance_mm_) {
      continue;
    }

    if (holes_only_ && !isHoleCylinder(solid, face, adaptor)) {
      continue;
    }

    CylinderFace cyl;
    cyl.radius = radius;
    cyl.axis = cylinder.Axis();
    cyl.area = faceArea(face);
    cyl.height = cylinderHeightEstimate(adaptor);
    raw.push_back(cyl);
  }

  return raw;
}

bool CylindricalHoleDetector::isHoleCylinder(
    const TopoDS_Shape& solid,
    const TopoDS_Face& face,
    const BRepAdaptor_Surface& adaptor) const {
  const double u =
      0.5 * (adaptor.FirstUParameter() + adaptor.LastUParameter());
  const double v =
      0.5 * (adaptor.FirstVParameter() + adaptor.LastVParameter());
  const gp_Pnt point_on_face = adaptor.Value(u, v);
  const gp_Pnt axis_point =
      projectPointToAxis(point_on_face, adaptor.Cylinder().Axis());

  BRepClass3d_SolidClassifier classifier(solid, axis_point, 1.0e-6);
  const TopAbs_State state = classifier.State();
  if (state == TopAbs_OUT) {
    return true;  // void on axis → cavity / hole
  }
  if (state == TopAbs_IN) {
    return false;  // material on axis → boss
  }
  // Ambiguous (ON): fall back to B-Rep orientation; inner faces are often REVERSED.
  return face.Orientation() == TopAbs_REVERSED;
}

double CylindricalHoleDetector::faceArea(const TopoDS_Face& face) {
  GProp_GProps props;
  BRepGProp::SurfaceProperties(face, props);
  return std::abs(props.Mass());
}

double CylindricalHoleDetector::cylinderHeightEstimate(
    const BRepAdaptor_Surface& adaptor) {
  const double u_mid =
      0.5 * (adaptor.FirstUParameter() + adaptor.LastUParameter());
  const double v0 = adaptor.FirstVParameter();
  const double v1 = adaptor.LastVParameter();
  const gp_Pnt p0 = adaptor.Value(u_mid, v0);
  const gp_Pnt p1 = adaptor.Value(u_mid, v1);
  return p0.Distance(p1);
}

gp_Pnt CylindricalHoleDetector::projectPointToAxis(const gp_Pnt& point,
                                                   const gp_Ax1& axis) {
  const gp_Pnt origin = axis.Location();
  const gp_Dir direction = axis.Direction();
  const gp_Vec vec(origin, point);
  const double t = vec.Dot(gp_Vec(direction.XYZ()));
  return gp_Pnt(origin.X() + direction.X() * t,
                origin.Y() + direction.Y() * t,
                origin.Z() + direction.Z() * t);
}

bool CylindricalHoleDetector::axesCompatible(const gp_Ax1& ax1,
                                             const gp_Ax1& ax2,
                                             double radius1,
                                             double radius2,
                                             double radius_tol,
                                             double angle_tol_deg,
                                             double distance_tol) {
  if (std::abs(radius1 - radius2) > radius_tol) {
    return false;
  }

  const gp_Dir d1 = ax1.Direction();
  const gp_Dir d2 = ax2.Direction();
  const double dot = std::abs(d1.Dot(d2));
  if (dot < std::cos(angle_tol_deg * kPi / 180.0)) {
    return false;
  }

  const gp_Pnt p1 = ax1.Location();
  const gp_Pnt p2 = ax2.Location();
  const gp_Vec vec(p1, p2);
  const gp_Vec cross = vec.Crossed(gp_Vec(d1));
  return cross.Magnitude() <= distance_tol;
}

std::vector<HoleInfo> CylindricalHoleDetector::mergeCoaxial(
    const std::vector<CylinderFace>& cylinders) const {
  const double radius_tol = tolerance_mm_ / 2.0;
  std::vector<std::vector<CylinderFace>> clusters;

  for (const CylinderFace& cyl : cylinders) {
    bool placed = false;
    for (auto& cluster : clusters) {
      const CylinderFace& ref = cluster.front();
      if (axesCompatible(ref.axis, cyl.axis, ref.radius, cyl.radius, radius_tol,
                         angle_tol_deg_, axis_distance_tol_mm_)) {
        cluster.push_back(cyl);
        placed = true;
        break;
      }
    }
    if (!placed) {
      clusters.push_back({cyl});
    }
  }

  std::vector<HoleInfo> holes;
  holes.reserve(clusters.size());

  for (const auto& cluster : clusters) {
    double radius_sum = 0.0;
    double area_sum = 0.0;
    double max_height = 0.0;
    const CylinderFace* ref = &cluster.front();

    for (const CylinderFace& c : cluster) {
      radius_sum += c.radius;
      area_sum += c.area;
      max_height = std::max(max_height, c.height);
      if (c.area > ref->area) {
        ref = &c;
      }
    }

    // Area-weighted average of axis locations → more stable hole coordinate.
    gp_XYZ weighted(0.0, 0.0, 0.0);
    double weight = 0.0;
    for (const CylinderFace& c : cluster) {
      const double w = std::max(c.area, 1.0e-9);
      weighted += c.axis.Location().XYZ() * w;
      weight += w;
    }
    weighted /= weight;

    HoleInfo hole;
    hole.radius_mm = radius_sum / static_cast<double>(cluster.size());
    hole.diameter_mm = hole.radius_mm * 2.0;
    hole.position = gp_Pnt(weighted);
    hole.raw_direction = ref->axis.Direction();
    hole.direction = snapToWorkpieceAxis(hole.raw_direction);
    hole.depth_mm = max_height;
    hole.face_count = static_cast<int>(cluster.size());
    hole.area_mm2 = area_sum;
    holes.push_back(hole);
  }

  return holes;
}

gp_Dir CylindricalHoleDetector::snapToWorkpieceAxis(const gp_Dir& direction) {
  // Align hole axis with the nearest workpiece principal axis (±X / ±Y / ±Z).
  const gp_Dir candidates[6] = {
      gp_Dir(1, 0, 0),  gp_Dir(-1, 0, 0), gp_Dir(0, 1, 0),
      gp_Dir(0, -1, 0), gp_Dir(0, 0, 1),  gp_Dir(0, 0, -1),
  };

  gp_Dir best = candidates[0];
  double best_dot = -1.0;
  for (const gp_Dir& cand : candidates) {
    const double dot = direction.Dot(cand);
    if (dot > best_dot) {
      best_dot = dot;
      best = cand;
    }
  }
  return best;
}

bool CylindricalHoleDetector::samplePointCloud() {
  cloud_->clear();
  cloud_->is_dense = false;

  BRepMesh_IncrementalMesh mesher(shape_, mesh_deflection_mm_, Standard_False,
                                  mesh_angular_deflection_, Standard_True);
  mesher.Perform();
  if (!mesher.IsDone()) {
    last_error_ = "BRep meshing failed";
    return false;
  }

  for (TopExp_Explorer explorer(shape_, TopAbs_FACE); explorer.More();
       explorer.Next()) {
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    TopLoc_Location location;
    Handle(Poly_Triangulation) triangulation =
        BRep_Tool::Triangulation(face, location);
    if (triangulation.IsNull()) {
      continue;
    }

    const gp_Trsf transform = location.Transformation();
    const Standard_Integer nb_nodes = triangulation->NbNodes();
    cloud_->points.reserve(cloud_->points.size() +
                           static_cast<std::size_t>(nb_nodes));

#if OCC_VERSION_HEX >= 0x070600
    for (Standard_Integer i = 1; i <= nb_nodes; ++i) {
      gp_Pnt p = triangulation->Node(i);
      p.Transform(transform);
      pcl::PointXYZ pt;
      pt.x = static_cast<float>(p.X());
      pt.y = static_cast<float>(p.Y());
      pt.z = static_cast<float>(p.Z());
      cloud_->points.push_back(pt);
    }
#else
    const TColgp_Array1OfPnt& nodes = triangulation->Nodes();
    for (Standard_Integer i = nodes.Lower(); i <= nodes.Upper(); ++i) {
      gp_Pnt p = nodes.Value(i);
      p.Transform(transform);
      pcl::PointXYZ pt;
      pt.x = static_cast<float>(p.X());
      pt.y = static_cast<float>(p.Y());
      pt.z = static_cast<float>(p.Z());
      cloud_->points.push_back(pt);
    }
#endif
  }

  cloud_->width = static_cast<std::uint32_t>(cloud_->points.size());
  cloud_->height = 1;
  return true;
}

bool CylindricalHoleDetector::savePointCloud(const std::string& pcd_path) const {
  if (!cloud_ || cloud_->empty()) {
    return false;
  }
  return pcl::io::savePCDFileBinaryCompressed(pcd_path, *cloud_) == 0;
}
