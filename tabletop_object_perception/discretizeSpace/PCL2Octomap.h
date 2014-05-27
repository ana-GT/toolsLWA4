/**
 * OctoMap ROS integration
 *
 * @author A. Hornung, University of Freiburg, Copyright (C) 2011-2012.
 * @see http://www.ros.org/wiki/octomap_ros
 * License: BSD
 */

/*
 * Copyright (c) 2011, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <octomap/octomap.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace octomap {
    
    /**
     * @brief Conversion from octomap::point3d_list (e.g. all occupied nodes from getOccupied()) to
     * pcl PointCloud
     *
     * @param points
     * @param scan
     */
    template <class PointT>
	static inline void pointsOctomapToPCL( const point3d_list& points, 
					       pcl::PointCloud<PointT>& cloud ){
	
	cloud.points.reserve(points.size());
	for (point3d_list::const_iterator it = points.begin(); it != points.end(); ++it){
	    cloud.push_back(PointT(it->x(), it->y(), it->z()));
	}
	
    }
    
    /**
     * @brief Conversion from a PCL pointcloud to octomap::Pointcloud, used internally in OctoMap
     *
     * @param pclCloud
     * @param octomapCloud
     */
    template <class PointT>
	static inline void pointcloudPCLToOctomap( const pcl::PointCloud<PointT>& pclCloud, 
						   Pointcloud& octomapCloud ){
	octomapCloud.reserve(pclCloud.points.size());
	
	typename pcl::PointCloud<PointT>::const_iterator it;
	for (it = pclCloud.begin(); it != pclCloud.end(); ++it){
	    // Check if the point is invalid
	    if (!std::isnan (it->x) && !std::isnan (it->y) && !std::isnan (it->z))
		octomapCloud.push_back(it->x, it->y, it->z);
	}
    }
    

  /// Conversion from pcl::PointT to octomap::point3d
  template <class PointT>
  static inline octomap::point3d pointPCLToOctomap(const PointT& p){
    return point3d(p.x, p.y, p.z);
  }

  /// Conversion from octomap::point3d to pcl::PointT, templated over pcl::PointT
  /// You might have to call this like "pointOctomapToPCL<pcl::PointXYZ>(point3d)"
  template <class PointT>
  static inline PointT pointOctomapToPCL(const point3d& octomapPt){
	return PointT(octomapPt.x(), octomapPt.y(), octomapPt.z());
  }






}


