#include "CylindricalHoleDetector.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

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

#if OCC_VERSION_HEX < 0x070600
#include <Poly_Array1OfTriangle.hxx>
#include <Poly_Triangle.hxx>
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;

/** Triangle area via cross product (CAD2PCD). */
float triangleArea(const pcl::PointXYZ& a,
                   const pcl::PointXYZ& b,
                   const pcl::PointXYZ& c) {
  const Eigen::Vector3f ab(b.x - a.x, b.y - a.y, b.z - a.z);
  const Eigen::Vector3f ac(c.x - a.x, c.y - a.y, c.z - a.z);
  return 0.5f * ab.cross(ac).norm();
}

/**
 * Area-weighted uniform surface sampling over a triangle mesh (CAD2PCD).
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleMeshSurface(
    const pcl::PolygonMesh& mesh,
    std::size_t num_samples) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *vertices);

  if (vertices->empty() || mesh.polygons.empty() || num_samples == 0) {
    return vertices;
  }

  std::vector<float> areas;
  areas.reserve(mesh.polygons.size());
  float total_area = 0.0f;

  for (const auto& poly : mesh.polygons) {
    if (poly.vertices.size() < 3) {
      areas.push_back(0.0f);
      continue;
    }
    const auto& a = vertices->points[poly.vertices[0]];
    const auto& b = vertices->points[poly.vertices[1]];
    const auto& c = vertices->points[poly.vertices[2]];
    const float area = triangleArea(a, b, c);
    areas.push_back(area);
    total_area += area;
  }

  if (total_area <= 0.0f) {
    return vertices;
  }

  std::vector<float> cdf(areas.size());
  float accum = 0.0f;
  for (std::size_t i = 0; i < areas.size(); ++i) {
    accum += areas[i] / total_area;
    cdf[i] = accum;
  }
  cdf.back() = 1.0f;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(num_samples);

  std::mt19937 rng(42);
  std::uniform_real_distribution<float> uniform(0.0f, 1.0f);

  for (std::size_t i = 0; i < num_samples; ++i) {
    const float r = uniform(rng);
    auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
    std::size_t tri_idx =
        static_cast<std::size_t>(std::distance(cdf.begin(), it));
    if (tri_idx >= mesh.polygons.size()) {
      tri_idx = mesh.polygons.size() - 1;
    }

    const auto& poly = mesh.polygons[tri_idx];
    if (poly.vertices.size() < 3) {
      continue;
    }

    const auto& A = vertices->points[poly.vertices[0]];
    const auto& B = vertices->points[poly.vertices[1]];
    const auto& C = vertices->points[poly.vertices[2]];

    float u = uniform(rng);
    float v = uniform(rng);
    if (u + v > 1.0f) {
      u = 1.0f - u;
      v = 1.0f - v;
    }
    const float w = 1.0f - u - v;

    pcl::PointXYZ p;
    p.x = w * A.x + u * B.x + v * C.x;
    p.y = w * A.y + u * B.y + v * C.y;
    p.z = w * A.z + u * B.z + v * C.z;
    cloud->push_back(p);
  }

  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

/**
 * Convert OCCT face triangulations into an in-memory PCL PolygonMesh.
 * No STL / disk I/O.
 */
bool shapeToPolygonMesh(const TopoDS_Shape& shape, pcl::PolygonMesh& mesh) {
  pcl::PointCloud<pcl::PointXYZ> vertices;
  std::vector<pcl::Vertices> polygons;

  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More();
       explorer.Next()) {
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    TopLoc_Location location;
    Handle(Poly_Triangulation) triangulation =
        BRep_Tool::Triangulation(face, location);
    if (triangulation.IsNull()) {
      continue;
    }

    const gp_Trsf transform = location.Transformation();
    const bool reversed = (face.Orientation() == TopAbs_REVERSED);
    const std::uint32_t index_offset =
        static_cast<std::uint32_t>(vertices.size());

#if OCC_VERSION_HEX >= 0x070600
    const Standard_Integer nb_nodes = triangulation->NbNodes();
    for (Standard_Integer i = 1; i <= nb_nodes; ++i) {
      gp_Pnt p = triangulation->Node(i);
      p.Transform(transform);
      pcl::PointXYZ pt;
      pt.x = static_cast<float>(p.X());
      pt.y = static_cast<float>(p.Y());
      pt.z = static_cast<float>(p.Z());
      vertices.push_back(pt);
    }

    const Standard_Integer nb_tris = triangulation->NbTriangles();
    for (Standard_Integer i = 1; i <= nb_tris; ++i) {
      Standard_Integer n1 = 0;
      Standard_Integer n2 = 0;
      Standard_Integer n3 = 0;
      triangulation->Triangle(i).Get(n1, n2, n3);
      if (reversed) {
        std::swap(n2, n3);
      }
      pcl::Vertices tri;
      tri.vertices = {index_offset + static_cast<std::uint32_t>(n1 - 1),
                      index_offset + static_cast<std::uint32_t>(n2 - 1),
                      index_offset + static_cast<std::uint32_t>(n3 - 1)};
      polygons.push_back(tri);
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
      vertices.push_back(pt);
    }

    const Poly_Array1OfTriangle& tris = triangulation->Triangles();
    const Standard_Integer node_lower = nodes.Lower();
    for (Standard_Integer i = tris.Lower(); i <= tris.Upper(); ++i) {
      Standard_Integer n1 = 0;
      Standard_Integer n2 = 0;
      Standard_Integer n3 = 0;
      tris.Value(i).Get(n1, n2, n3);
      if (reversed) {
        std::swap(n2, n3);
      }
      pcl::Vertices tri;
      tri.vertices = {
          index_offset + static_cast<std::uint32_t>(n1 - node_lower),
          index_offset + static_cast<std::uint32_t>(n2 - node_lower),
          index_offset + static_cast<std::uint32_t>(n3 - node_lower)};
      polygons.push_back(tri);
    }
#endif
  }

  if (vertices.empty() || polygons.empty()) {
    return false;
  }

  vertices.width = static_cast<std::uint32_t>(vertices.size());
  vertices.height = 1;
  vertices.is_dense = true;
  pcl::toPCLPointCloud2(vertices, mesh.cloud);
  mesh.polygons = std::move(polygons);
  return true;
}

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

void CylindricalHoleDetector::setSampleCount(std::size_t num_samples) {
  sample_count_ = num_samples;
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

  // 1) Mesh B-Rep in memory
  BRepMesh_IncrementalMesh mesher(shape_, mesh_deflection_mm_, Standard_False,
                                  mesh_angular_deflection_, Standard_True);
  mesher.Perform();
  if (!mesher.IsDone()) {
    last_error_ = "BRep meshing failed";
    return false;
  }

  // 2) OCCT triangulation → PCL PolygonMesh (no STL file I/O)
  pcl::PolygonMesh mesh;
  if (!shapeToPolygonMesh(shape_, mesh)) {
    last_error_ = "Failed to build in-memory triangle mesh from B-Rep";
    return false;
  }

  // 3) CAD2PCD area-weighted surface sampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr sampled =
      sampleMeshSurface(mesh, sample_count_);
  if (!sampled || sampled->empty()) {
    last_error_ = "Mesh surface sampling produced an empty cloud";
    return false;
  }

  *cloud_ = *sampled;
  cloud_->width = static_cast<std::uint32_t>(cloud_->size());
  cloud_->height = 1;
  cloud_->is_dense = true;
  return true;
}

bool CylindricalHoleDetector::savePointCloud(const std::string& pcd_path) const {
  if (!cloud_ || cloud_->empty()) {
    return false;
  }
  return pcl::io::savePCDFileBinaryCompressed(pcd_path, *cloud_) == 0;
}
