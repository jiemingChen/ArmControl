//
// Created by jieming on 03.12.20.
//

#ifndef PANDA_CONTROL_COLLISION_DETECTION_H
#define PANDA_CONTROL_COLLISION_DETECTION_H

#include "common.h"
#include <iostream>
#include <Eigen/Eigen>
#include <fcl/data_types.h>
#include <igl/readSTL.h>
#include <igl/readOBJ.h>

namespace ReadMesh
{
    void loadSTLFile(const char* filename, vector<fcl::Vec3f >& points, vector<fcl::Triangle>& triangles);
     void loadModel(const char* filename, vector<fcl::Vec3f>& points,  vector<fcl::Triangle>& triangles);
}



#endif //PANDA_CONTROL_COLLISION_DETECTION_H
