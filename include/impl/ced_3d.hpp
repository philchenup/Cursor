#ifndef CED_3D_HPP_
#define CED_3D_HPP_

template<typename PointInT, typename PointOutT> bool
pcl::CEDKeypoint3D<PointInT, PointOutT>::initCompute ()
{
  if (!Keypoint<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] init failed!\n", name_.c_str ());
    return (false);
  }
  if (non_max_radius_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : the non maxima radius (%f) must be strict positive!\n",
        name_.c_str (), non_max_radius_);
    return (false);
  }
  if (min_neighbors_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : the minimum number of neighbors (%d) must be strict positive!\n",
        name_.c_str (), min_neighbors_);
    return (false);
  }
  return (true);
}


template<typename PointInT, typename PointOutT> void
pcl::CEDKeypoint3D<PointInT, PointOutT>::detectKeypoints (PointCloudOut &output)
{
  output.clear ();
  const int input_size = static_cast<int> (input_->size ());
  std::vector<float> geo_centroid_norm (static_cast<std::size_t> (input_size), 0.0f);

  for (int idx = 0; idx < input_size; ++idx)
  {
    pcl::Indices nn_indices;
    std::vector<float> nn_dists;
    tree_->radiusSearch (idx, search_radius_, nn_indices, nn_dists);

    if (static_cast<int> (nn_indices.size ()) < min_neighbors_)
    {
      geo_centroid_norm[static_cast<std::size_t> (idx)] = 0.0f;
      continue;
    }

    Eigen::Vector3f geo_centroid = Eigen::Vector3f::Zero ();
    for (const auto& nn_idx : nn_indices)
      geo_centroid += (*input_)[nn_idx].getVector3fMap ();
    geo_centroid /= static_cast<float> (nn_indices.size ());
    geo_centroid_norm[static_cast<std::size_t> (idx)] =
        (geo_centroid - (*input_)[idx].getVector3fMap ()).norm ();
  }

  for (int idx = 0; idx < input_size; ++idx)
  {
    if (geo_centroid_norm[static_cast<std::size_t> (idx)] < centroid_threshold_ * search_radius_)
      continue;

    pcl::Indices nn_indices;
    std::vector<float> nn_dists;
    tree_->radiusSearch (idx, non_max_radius_, nn_indices, nn_dists);

    bool maximum = true;
    for (const auto& nn_idx : nn_indices)
    {
      if (geo_centroid_norm[static_cast<std::size_t> (idx)] <
          geo_centroid_norm[static_cast<std::size_t> (nn_idx)])
      {
        maximum = false;
        break;
      }
    }
    if (maximum)
    {
      PointOutT p;
      p.getVector3fMap () = (*input_)[idx].getVector3fMap ();
      output.push_back (p);
      keypoints_indices_->indices.push_back (idx);
    }
  }

  output.height = 1;
  output.width = static_cast<std::uint32_t> (output.size ());
  output.is_dense = input_->is_dense;
}

#endif  // CED_3D_HPP_
