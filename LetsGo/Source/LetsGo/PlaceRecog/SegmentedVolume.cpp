#include "SegmentedVolume.h"



SegmentedVolume::SegmentedVolume()
{
}

std::vector<std::vector<Eigen::Vector3f>> SegmentedVolume::Clustering(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_pc_in, Eigen::Vector3f unit_size,
	float z_axis_min_, float z_axis_max_, int cluster_size_min_, int cluster_size_max_, int regions_dist,
	float tolerance_, float dist_filter_weight_, float tolerance_weight_)
{

	const int region_max_ = 5;
	int regions_[100];
	for (int i = 0; i < 100; i++)
		regions_[i] = regions_dist;

	// Point Cloud의 z축 영역 필터링(z_axis_min_ ~ z_axis_max_)
	pcl::IndicesPtr pc_indices(new std::vector<int>);
	pcl::PassThrough<pcl::PointXYZI> pt;
	pt.setInputCloud(pcl_pc_in);
	pt.setFilterFieldName("z");
	pt.setFilterLimits(z_axis_min_, z_axis_max_);
	pt.filter(*pc_indices);

	// nested circular regions로 포인트를 계층 분리
	boost::array<std::vector<int>, region_max_> indices_array; //
	for (int i = 0; i < pc_indices->size(); i++)
	{
		float range = 0.0;
		for (int j = 0; j < region_max_; j++)
		{
			pcl::PointXYZI p = pcl_pc_in->points[(*pc_indices)[i]];
			float dist = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);

			if (dist > range && dist <= (range + regions_[j]))
			{
				indices_array[j].push_back((*pc_indices)[i]);
				break;
			}
			range += regions_[j];
		}
	}


	// Euclidean clustering
	float tolerance = tolerance_;
	float k_merging_threshold_ = 300;
	//int last_clusters_begin = 0;
	//int last_clusters_end = 0;
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;
	for (int i = 0; i < region_max_; i++)
	{
		//tolerance += tolerance_add_;
		int szIndicesArr = indices_array[i].size();
		if (szIndicesArr > cluster_size_min_)
		{
			boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));

			std::vector<pcl::PointIndices> cluster_indices;
			Extract(cluster_indices, tolerance, cluster_size_min_, cluster_size_max_,
				dist_filter_weight_, tolerance_weight_, pcl_pc_in, indices_array_ptr);

			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++)
			{
				pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
				for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
					cluster->points.push_back(pcl_pc_in->points[*pit]);
				}

				/*** Merge clusters separated by nested regions ***/
				/*
				for (int j = last_clusters_begin; j < last_clusters_end; j++)
				{
					int K = 1; //the number of neighbors to search for
					std::vector<int> k_indices(K);
					std::vector<float> k_sqr_distances(K);
					treeFLANN.setInputCloud(cluster);
					if (clusters[j]->points.size() >= 1)
					{
						if (treeFLANN.nearestKSearch(*clusters[j], clusters[j]->points.size() - 1, K, k_indices, k_sqr_distances) > 0)
						{
							if (std::sqrt(k_sqr_distances[0]) < k_merging_threshold_)
							{
								*cluster += *clusters[j];
								clusters.erase(clusters.begin() + j);
								last_clusters_end--;
								PRINTF("k-merging: clusters %d is merged.", j);
							}
						}
					}
				}
				*/

				cluster->width = cluster->size();
				cluster->height = 1;
				cluster->is_dense = true;
				clusters.push_back(cluster);

				//last_clusters_begin = last_clusters_end;
				//last_clusters_end = clusters.size();
			}


		}
	}

	//세그먼트 박스 데이터 추출
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::copyPointCloud(*pcl_pc_in, *pc_indices, *pcl_pc_out);
	std::vector<Eigen::Vector4f> boxMins;
	std::vector<Eigen::Vector4f> boxMaxs;
	for (int i = 0; i < clusters.size(); i++)
	{
		Eigen::Vector4f min, max;
		pcl::getMinMax3D(*clusters[i], min, max);

		boxMins.push_back(min);
		boxMaxs.push_back(max);
	}

	//최종 박스 출력
	std::vector<std::vector<Eigen::Vector3f>> results;
	for (int i = 0; i < boxMins.size(); i++)
	{
		std::vector<Eigen::Vector3f> box;
		Eigen::Vector4f min = boxMins[i];
		Eigen::Vector4f max = boxMaxs[i];

		box.push_back(Eigen::Vector3f(min[0], min[1], min[2]));
		box.push_back(Eigen::Vector3f(max[0], min[1], min[2]));
		box.push_back(Eigen::Vector3f(max[0], max[1], min[2]));
		box.push_back(Eigen::Vector3f(min[0], max[1], min[2]));
		box.push_back(Eigen::Vector3f(min[0], min[1], max[2]));
		box.push_back(Eigen::Vector3f(max[0], min[1], max[2]));
		box.push_back(Eigen::Vector3f(max[0], max[1], max[2]));
		box.push_back(Eigen::Vector3f(min[0], max[1], max[2]));

		results.push_back(box);
	}

	return results;
}

void SegmentedVolume::Extract(std::vector<pcl::PointIndices> &clusters,
	float cluster_tolerance_, int min_pts_per_cluster_, int max_pts_per_cluster_,
	float dist_filter_weight_, float tolerance_weight_,
	const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input_, const pcl::IndicesPtr &indices_)
{
	if ((input_ != 0 && input_->points.empty()) || (indices_ != 0 && indices_->empty()))
	{
		clusters.clear();
		return;
	}

	// Send the input dataset to the spatial locator
	extractEUClusters(*input_, *indices_, static_cast<float> (cluster_tolerance_), clusters, min_pts_per_cluster_, max_pts_per_cluster_, dist_filter_weight_, tolerance_weight_);

	// Sort the clusters based on their size (largest one first)
	std::sort(clusters.rbegin(), clusters.rend(), comparePointClusters);
}

void SegmentedVolume::extractEUClusters(const pcl::PointCloud<pcl::PointXYZI> &cloud, const std::vector<int> &indices,
	float tolerance, std::vector<pcl::PointIndices> &clusters,
	unsigned int min_pts_per_cluster, unsigned int max_pts_per_cluster,
	float dist_filter_weight, float tolerance_weight)
{
	// Create a bool vector of processed point indices, and initialize it to false
	std::vector<bool> processed(cloud.points.size(), false);

	std::vector<int> nn_indices;
	std::vector<float> nn_distances;
	// Process all points in the indices vector
	for (int i = 0; i < static_cast<int> (indices.size()); ++i)
	{
		if (processed[indices[i]])
			continue;

		std::vector<int> seed_queue;
		int sq_idx = 0;
		seed_queue.push_back(indices[i]);

		processed[indices[i]] = true;

		while (sq_idx < static_cast<int> (seed_queue.size()))
		{
			//Radius search
			int ret = 0;
			float weightedAvgDist = tolerance;
			pcl::PointXYZI p1 = cloud.points[seed_queue[sq_idx]];
			for (int n = 0; n < indices.size(); n++)
			{
				if (n == seed_queue[sq_idx])
					continue;

				pcl::PointXYZI p2 = cloud.points[indices[n]];
				float dist = sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z));

				if (dist < weightedAvgDist)
				{
					nn_indices.push_back(indices[n]);
					nn_distances.push_back(dist);

					ret++;
					weightedAvgDist = ((1.0 - dist_filter_weight) * weightedAvgDist) + (dist_filter_weight * dist);
					weightedAvgDist = ((1.0 - tolerance_weight) * tolerance) + (tolerance_weight * weightedAvgDist);
				}

			}

			if (ret == -1)
			{
				PCL_ERROR("[pcl::extractEuclideanClusters] Received error code -1 from radiusSearch\n");
				exit(0);
			}
			if (!ret)
			{
				sq_idx++;
				continue;
			}

			for (size_t j = 0; j < nn_indices.size(); ++j)             // can't assume sorted (default isn't!)
			{
				if (nn_indices[j] == -1 || processed[nn_indices[j]])        // Has this point been processed before ?
					continue;

				// Perform a simple Euclidean clustering
				seed_queue.push_back(nn_indices[j]);
				processed[nn_indices[j]] = true;
			}

			sq_idx++;
		}

		// If this queue is satisfactory, add to the clusters
		if (seed_queue.size() >= min_pts_per_cluster && seed_queue.size() <= max_pts_per_cluster)
		{
			pcl::PointIndices r;
			r.indices.resize(seed_queue.size());
			for (size_t j = 0; j < seed_queue.size(); ++j)
				// This is the only place where indices come into play
				r.indices[j] = seed_queue[j];

			// These two lines should not be needed: (can anyone confirm?) -FF
			//r.indices.assign(seed_queue.begin(), seed_queue.end());
			std::sort(r.indices.begin(), r.indices.end());
			r.indices.erase(std::unique(r.indices.begin(), r.indices.end()), r.indices.end());

			r.header = cloud.header;
			clusters.push_back(r);   // We could avoid a copy by working directly in the vector
		}
	}
}

bool SegmentedVolume::comparePointClusters(const pcl::PointIndices &a, const pcl::PointIndices &b)
{
	return (a.indices.size() < b.indices.size());
}