// Copyright (c) 2024 Julian Gaal
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <memory>
#include "nanoflann.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

static_assert(NANOFLANN_VERSION == 0x150 ||     //
                  NANOFLANN_VERSION == 0x151 || //
                  NANOFLANN_VERSION == 0x152 || //
                  NANOFLANN_VERSION == 0x153,   //
              "Unsupported nanoflann version. Supported are version 1.5.0-1.5.3");

namespace nanoflann {

/**
 * @brief Struct mirroring the PCL FLANN interface, providing knn and radius search.
 */
template <typename PointT> class KdTreeFLANN {
  public:
    using PointCloud = typename pcl::PointCloud<PointT>;
    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;
    using PointCloudConstPtr = typename pcl::PointCloud<PointT>::ConstPtr;

    using Ptr = std::shared_ptr<KdTreeFLANN<PointT>>;
    using ConstPtr = std::shared_ptr<const KdTreeFLANN<PointT>>;
    using IndicesPtr = std::shared_ptr<std::vector<int>>;
    using IndicesConstPtr = std::shared_ptr<const std::vector<int>>;

    KdTreeFLANN(bool sorted = true, size_t leaf_max_size = 25, unsigned int n_thread_build = 1);

    ~KdTreeFLANN() = default;

    /**
     * @brief search for eps-approximate neighbours (default: 0)
     * @param eps The epsilon value to set.
     */
    inline void setEpsilon(float eps) { _params.eps = eps; }

    /**
     * @brief Sets whether the results should be sorted.
     * @param sorted Boolean indicating whether to sort the results.
     */
    inline void setSortedResults(bool sorted) { _params.sorted = sorted; }

    /**
     * @brief Sets the input point cloud for processing.
     * @param cloud Constant pointer to the input point cloud.
     * @param indices (Optional) constant pointer to indices for queries against the input cloud.
     */
    void setInputCloud(const PointCloudConstPtr& cloud, const IndicesConstPtr& indices = IndicesConstPtr());

    /**
     * @brief Returns a constant pointer to the input point cloud.
     * @return Constant pointer to the input point cloud.
     */
    inline PointCloudConstPtr getInputCloud() const { return _adaptor.pcl; }

    /**
     * @brief Finds the k-nearest neighbors of a given point in the point cloud.
     *
     * This function identifies the k-nearest neighbors of the specified point
     * in the point cloud. The indices and squared distances of the nearest points
     * are returned in the provided vectors.
     *
     * @param point The query point for the nearest neighbors search.
     * @param k The number of nearest neighbors to find.
     * @param k_indices A vector to store the indices of the k-nearest neighbors.
     * @param k_sqr_distances A vector to store the squared distances of the k-nearest neighbors.
     *
     * @return The number of nearest neighbors found (may be less than k if not enough
     *         neighbors are available in the point cloud).
     *
     */
    int nearestKSearch(const PointT& point, int k, std::vector<int>& k_indices,
                       std::vector<float>& k_sqr_distances) const;

    /**
     * @brief Performs a radius search around a given point in the point cloud.
     *
     * This function finds all points within a specified radius from the given point
     * in the point cloud. The indices and squared distances of the neighboring points
     * are returned in the provided vectors.
     *
     * @param point The query point for the radius search.
     * @param radius The radius within which points are considered neighbors, in meters.
     * @param k_indices A vector to store the indices of neighboring points.
     * @param k_sqr_distances A vector to store the squared distances of neighboring points.
     *
     * @return return point indices and squared distances found within the specified radius.
     */
    int radiusSearch(const PointT& point, float radius, std::vector<int>& k_indices,
                     std::vector<float>& k_sqr_distances) const;

  protected:
    nanoflann::SearchParameters _params;

    /**
     * @brief Struct representing a PCL point cloud adaptor, providing knn and radius search.
     * Matches this interface https://jlblancoc.github.io/nanoflann/structnanoflann_1_1KDTreeEigenMatrixAdaptor.html
     */
    struct PointCloud_Adaptor {

        PointCloud_Adaptor() : pcl(), indices() {}

        ~PointCloud_Adaptor() = default;
        /**
         * @brief Gets the point count for the k-d tree.
         * @return The number of points in the k-d tree.
         */
        inline size_t kdtree_get_point_count() const;

        /**
         * @brief Gets the value of a point at a specified index and dimension.
         * @param idx Index of the point.
         * @param dim Dimension of the point. (in S03 0, 1, or 2)
         * @return The value of the point at the specified index and dimension.
         */
        inline float kdtree_get_pt(const size_t idx, int dim) const;

        /**
         * @brief Gets the bounding box for the k-d tree (template implementation).
         * @tparam BBOX The bounding box type.
         * @param[out] bbox The bounding box to be filled.
         * @return Always returns false: Nanoflann assumes adaptor needs no special handling
         */
        template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }

        PointCloudConstPtr pcl;

        IndicesConstPtr indices;
    };

    using KDTreeFlann_PCL_SO3 = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::SO3_Adaptor<float, PointCloud_Adaptor>,
                                                                    PointCloud_Adaptor, 3, int>;

    PointCloud_Adaptor _adaptor;

    KDTreeFlann_PCL_SO3 _kdtree;
};

template <typename PointT>
inline KdTreeFLANN<PointT>::KdTreeFLANN(bool sorted, size_t leaf_max_size, unsigned int n_thread_build)
    : _params(), _adaptor(), _kdtree(3, _adaptor, KDTreeSingleIndexAdaptorParams(leaf_max_size, KDTreeSingleIndexAdaptorFlags::None, n_thread_build)) {
    _params.sorted = sorted;
}

template <typename PointT>
inline void KdTreeFLANN<PointT>::setInputCloud(const KdTreeFLANN::PointCloudConstPtr& cloud,
                                               const IndicesConstPtr& indices) {
    _adaptor.pcl = cloud;
    _adaptor.indices = indices;
    _kdtree.buildIndex();
}

template <typename PointT>
inline int KdTreeFLANN<PointT>::nearestKSearch(const PointT& point, int num_closest, std::vector<int>& k_indices,
                                               std::vector<float>& k_sqr_distances) const {
    k_indices.resize(num_closest);
    k_sqr_distances.resize(num_closest);

    nanoflann::KNNResultSet<float, int> resultSet(num_closest);
    resultSet.init(k_indices.data(), k_sqr_distances.data());
    _kdtree.findNeighbors(resultSet, point.data, _params);
    return resultSet.size();
}

template <typename PointT>
inline int KdTreeFLANN<PointT>::radiusSearch(const PointT& point, float radius, std::vector<int>& k_indices,
                                             std::vector<float>& k_sqr_distances) const {
    static std::vector<nanoflann::ResultItem<int, float>> result;
    result.reserve(256);

    // From nanoflann README
    // > Important note: If L2 norms are used, notice that search radius and all passed and returned      distances are
    // actually squared distances.
    //
    // I argue that the user expects searches in m. Therefore square the radius for the user.
    //
    // Earlier versions of this lbrary added tiny offset to effectively search <= radius, but
    // this does not correspond to the behavior in pcl, therfore remove offset
    float sq_radius = radius * radius; // + 0.05

    const size_t n_found = _kdtree.radiusSearch(point.data, sq_radius, result);

    k_indices.resize(n_found);
    k_sqr_distances.resize(n_found);
    for (size_t i = 0; i < n_found; i++) {
        k_indices[i] = result[i].first;
        k_sqr_distances[i] = result[i].second;
    }
    return n_found;
}

template <typename PointT> inline size_t KdTreeFLANN<PointT>::PointCloud_Adaptor::kdtree_get_point_count() const {
    if (indices) {
        return indices->size();
    }
    if (pcl) {
        return pcl->points.size();
    }
    return 0;
}

template <typename PointT>
inline float KdTreeFLANN<PointT>::PointCloud_Adaptor::kdtree_get_pt(const size_t idx, int dim) const {
    const PointT& p = (indices) ? pcl->points[(*indices)[idx]] : pcl->points[idx];
    if (dim == 0) {
        return p.x;
    } else if (dim == 1) {
        return p.y;
    } else {
        return p.z;
    }

    return 0.0;
}

} // namespace nanoflann