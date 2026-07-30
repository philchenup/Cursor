#pragma once

/**
 * CylindricalHoleDetector
 *
 * Detect cylindrical holes in a STEP B-Rep model using Open CASCADE Technology
 * (OCCT >= 7.4), sample the model surface into a PCL point cloud, and report
 * each hole's center and principal-axis-aligned direction (workpiece X/Y/Z).
 *
 * Pipeline:
 *   1. Read STEP → B-Rep shape
 *   2. Collect cylindrical faces matching target diameter ± tolerance
 *   3. Classify hole vs boss via solid classification on the cylinder axis
 *   4. Cluster coaxial cylinders of similar radius into logical holes
 *   5. Snap each hole axis to the nearest workpiece ±X / ±Y / ±Z
 *   6. Mesh the shape and emit a PCL PointXYZ cloud
 */

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <BRepAdaptor_Surface.hxx>
#include <Standard_Version.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#if OCC_VERSION_HEX < 0x070400
#error "CylindricalHoleDetector requires OpenCASCADE 7.4.0 or newer"
#endif

struct HoleInfo {
  double diameter_mm = 0.0;
  double radius_mm = 0.0;
  /** Representative point on the hole axis (mm, model coordinates). */
  gp_Pnt position;
  /**
   * Hole axis direction snapped to the nearest workpiece principal axis
   * (±X, ±Y or ±Z). Unit vector.
   */
  gp_Dir direction;
  /** Raw geometric axis before principal-axis snapping. */
  gp_Dir raw_direction;
  /** Estimated axial depth from cylindrical face UV span (mm). */
  double depth_mm = 0.0;
  int face_count = 0;
  double area_mm2 = 0.0;
};

class CylindricalHoleDetector {
 public:
  /**
   * @param step_path        Path to .step / .stp file
   * @param diameter_mm      Target hole diameter in millimetres
   * @param tolerance_mm     Absolute diameter tolerance (default 0.05 mm)
   */
  CylindricalHoleDetector(std::string step_path,
                          double diameter_mm,
                          double tolerance_mm = 0.05);

  ~CylindricalHoleDetector() = default;

  CylindricalHoleDetector(const CylindricalHoleDetector&) = delete;
  CylindricalHoleDetector& operator=(const CylindricalHoleDetector&) = delete;

  /** Linear deflection for BRep meshing (mm). Smaller → denser cloud. */
  void setMeshDeflection(double deflection_mm);
  /** Angular deflection for BRep meshing (radians). */
  void setMeshAngularDeflection(double angle_rad);
  /** If true (default), discard outer cylindrical bosses. */
  void setHolesOnly(bool holes_only);
  /** Clustering tolerances. */
  void setAngleToleranceDeg(double angle_tol_deg);
  void setAxisDistanceTolerance(double distance_tol_mm);

  /**
   * Load STEP, detect holes, build PCL cloud.
   * @return true on success
   */
  bool process();

  const std::string& lastError() const { return last_error_; }

  const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloud() const {
    return cloud_;
  }

  const std::vector<HoleInfo>& holes() const { return holes_; }

  const TopoDS_Shape& shape() const { return shape_; }

  /** Save cloud as PCD (binary compressed). Returns false on failure. */
  bool savePointCloud(const std::string& pcd_path) const;

 private:
  struct CylinderFace {
    double radius = 0.0;
    gp_Ax1 axis;
    double area = 0.0;
    double height = 0.0;
  };

  bool loadStep();
  TopoDS_Shape extractSolid(const TopoDS_Shape& shape) const;
  std::vector<CylinderFace> collectCylinderFaces(const TopoDS_Shape& solid) const;
  bool isHoleCylinder(const TopoDS_Shape& solid,
                      const TopoDS_Face& face,
                      const BRepAdaptor_Surface& adaptor) const;
  static double faceArea(const TopoDS_Face& face);
  static double cylinderHeightEstimate(const BRepAdaptor_Surface& adaptor);
  static gp_Pnt projectPointToAxis(const gp_Pnt& point, const gp_Ax1& axis);
  static bool axesCompatible(const gp_Ax1& ax1,
                             const gp_Ax1& ax2,
                             double radius1,
                             double radius2,
                             double radius_tol,
                             double angle_tol_deg,
                             double distance_tol);
  std::vector<HoleInfo> mergeCoaxial(const std::vector<CylinderFace>& cylinders) const;
  static gp_Dir snapToWorkpieceAxis(const gp_Dir& direction);
  bool samplePointCloud();

  std::string step_path_;
  double diameter_mm_;
  double tolerance_mm_;
  double mesh_deflection_mm_ = 0.5;
  double mesh_angular_deflection_ = 0.5;
  bool holes_only_ = true;
  double angle_tol_deg_ = 2.0;
  double axis_distance_tol_mm_ = 0.1;

  TopoDS_Shape shape_;
  std::vector<HoleInfo> holes_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  std::string last_error_;
};
