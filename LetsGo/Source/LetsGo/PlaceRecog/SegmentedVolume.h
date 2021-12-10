#pragma once

#pragma warning(disable:4643)
#include "LetsGo.h"
#include "pcl/conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/common.h"
#include "pcl/common/centroid.h"
#include "boost/array.hpp"
#include <algorithm>

class SegmentedVolume
{
public:
	SegmentedVolume();

public:
	std::vector<std::vector<Eigen::Vector3f>> Clustering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_pc_in, Eigen::Vector3f unit_size,
		float z_axis_min_, float z_axis_max_, int cluster_size_min_, int cluster_size_max_, int regions_dist,
		float tolerance_, float dist_filter_weight_, float tolerance_weight_);

	void Extract(std::vector<pcl::PointIndices> &clusters,
		float cluster_tolerance_, int min_pts_per_cluster_, int max_pts_per_cluster_,
		float dist_filter_weight_, float tolerance_weight_,
		const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input_, const pcl::IndicesPtr &indices_);

	void extractEUClusters(const pcl::PointCloud<pcl::PointXYZI> &cloud, const std::vector<int> &indices,
		float tolerance, std::vector<pcl::PointIndices> &clusters,
		unsigned int min_pts_per_cluster, unsigned int max_pts_per_cluster,
		float dist_filter_weight, float tolerance_weight);

	static bool comparePointClusters(const pcl::PointIndices &a, const pcl::PointIndices &b);
public:
	pcl::KdTreeFLANN<pcl::PointXYZI> treeFLANN;
};

