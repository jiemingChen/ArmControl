#ifndef BIRRT_STAR_ALGORITHM_COLLISION_CHECKER_HPP_
#define BIRRT_STAR_ALGORITHM_COLLISION_CHECKER_HPP_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdint.h>
#include <map>
#include <tuple>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <fcl/collision_object.h>
#include <urdf_parser/urdf_parser.h>
#include <fcl/config.h>
#include <fcl/collision.h>
#include <fcl/octree.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/BVH/BVH_model.h>
#include <geometric_shapes/mesh_operations.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>

namespace birrt_star_motion_planning
{

class CollisionChecker
{
  typedef double Real;
  typedef uint32_t UInt;
  typedef int32_t Int;

  struct Link
  {
    KDL::Frame transform;
    KDL::Frame *transformParent;
    std::string name;

    KDL::Frame transformToParent;
    const KDL::Joint* joint;
    Real* jointValue;

    fcl::CollisionObject* collisionObject;
    std::string mesh;
    bool isInCollision;

    Link() :
        collisionObject(NULL), transformParent(NULL), joint(NULL), jointValue(NULL)
    {
    }
  };

public:
  /**
   * @brief Constructor. Initializes everything.
   */
  CollisionChecker() :
      initialized(false), octomapCollisionObject(NULL)
  {
    if (!initialize())
    {
      ros::shutdown();
      return;
    }
    initialized = true;
  }

  void setOcTree(const octomap::OcTree* octree)
  {
    if (octree == NULL)
      return;

    const boost::shared_ptr<const octomap::OcTree> octreePtr = boost::make_shared<const octomap::OcTree>(*octree);
    boost::shared_ptr<fcl::CollisionGeometry> octomapCollisionGeometry(new fcl::OcTree(octreePtr));

    if (octomapCollisionObject)
      delete octomapCollisionObject;
    octomapCollisionObject = new fcl::CollisionObject(octomapCollisionGeometry);
    octomapCollisionObject->setTransform(fcl::Quaternion3f(1, 0, 0, 0), fcl::Vec3f(0, 0, -0.02));
  }

  void setDisabledLinkMapCollisions(const std::vector<std::string> &linksDisabled)
  {
    if (linksDisabled.size() == 0 && octomapCollisionLinks.size() == links.size())
      checkAllOcotmapLinkCollisions = true;
    else
    {
      checkAllOcotmapLinkCollisions = false;
      for (std::map<std::string, std::pair<Link*, bool> >::iterator it = octomapCollisionLinks.begin(); it != octomapCollisionLinks.end(); ++it)
        it->second.second = true;
      for (std::vector<std::string>::const_iterator it = linksDisabled.begin(); it != linksDisabled.end(); ++it)
        octomapCollisionLinks[*it].second = false;
    }
  }

  bool isInCollision(const std::vector<Real> &jointPositions, const bool checkSelfCollision, const bool checkMapCollision)
  {
    if (!initialized)
      return true;

    if (!checkSelfCollision && !checkMapCollision)
      return false;

    updateTransforms(jointPositions);

    if (checkMapCollision && this->checkMapCollision())
      return true;

    if (checkSelfCollision && this->checkSelfCollision())
      return true;

    return false;
  }

  void getCollisions(const std::vector<Real> &jointPositions, std::vector<std::pair<std::string, std::string> > &selfCollisions,
                     std::vector<std::string> &mapCollisions)
  {
    updateTransforms(jointPositions);

    for (std::vector<Link>::iterator it = links.begin(); it != links.end(); ++it)
      it->isInCollision = false;
    getSelfCollisions(selfCollisions);
    getMapCollisions(mapCollisions);
    publishCollisionModell();
  }

private:
  bool initialized;
  ros::NodeHandle nh;
  ros::Publisher publisherCollisionModell;

  KDL::Tree kdlTree;
  std::vector<Real> jointPositions;
  std::vector<Link> links;

  std::vector<std::pair<Link*, Link*> > selfCollisionPairs;
  std::map<std::string, std::pair<Link*, bool> > octomapCollisionLinks;
  bool checkAllOcotmapLinkCollisions;

  fcl::CollisionObject* octomapCollisionObject;

  // ******************** INITIALIZATION ********************

  /**
   * @brief Initializes all solvers and the reachability map.
   * @return Returns false if the robot description cannot be read or the defined chains are not available.
   */
  bool initialize()
  {
    if (!initializeKDL())
      return false;

    if (!initializeFCL())
      return false;

    checkAllOcotmapLinkCollisions = true;
    findSelfCollisionPairs();

    publisherCollisionModell = nh.advertise<visualization_msgs::MarkerArray>("collision_modell", -1);

    return true;
  }

  /**
   * @brief Initializes the chains that are used for the IK solvers.
   * @return Returns false if the defined chains are not available.
   */
  bool initializeKDL()
  {
    bool readFine = kdl_parser::treeFromParam("/squirrel_8dof_planning/robot_description", kdlTree);
    if (!readFine)
    {
      ROS_ERROR("Could not parse urdf.");
      return false;
    }

    jointPositions.resize(8, 0.0);
    createLinksKDL();

    for (std::vector<Link>::iterator it = links.begin(); it != links.end(); ++it)
      octomapCollisionLinks[it->name] = std::make_pair(&(*it), true);
    std::cout << "Loaded " << links.size() << " links into the collision modell." << std::endl;

    return true;
  }

  void createLinksKDL()
  {
    links.reserve(100);
    links.push_back(Link());
    links.back().name = kdlTree.getRootSegment()->second.segment.getName();
    links.back().transform = KDL::Frame::Identity();
    links.back().transform.p.data[2] = 0.02;
    links.back().transformToParent = KDL::Frame::Identity();

    expandTreeKDL(kdlTree.getRootSegment(), 0);
  }

  void expandTreeKDL(const KDL::SegmentMap::const_iterator& segment, UInt indexParent)
  {
    for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator child = segment->second.children.begin(); child != segment->second.children.end();
        ++child)
    {
      links.push_back(Link());
      links.back().name = (*child)->second.segment.getName();
      links.back().transformParent = &links[indexParent].transform;

      if (links.back().name == "base_x_link")
      {
        links.back().joint = &((*child)->second.segment.getJoint());
        links.back().jointValue = &jointPositions[0];
      }
      else if (links.back().name == "base_y_link")
      {
        links.back().joint = &((*child)->second.segment.getJoint());
        links.back().jointValue = &jointPositions[1];
      }
      else if (links.back().name == "base_theta_link")
      {
        links.back().joint = &((*child)->second.segment.getJoint());
        links.back().jointValue = &jointPositions[2];
      }
      else if (links.back().name == "arm_link1")
      {
        links.back().joint = &((*child)->second.segment.getJoint());
        links.back().jointValue = &jointPositions[3];
      }
      else if (links.back().name == "arm_motor2")
      {
        links.back().joint = &((*child)->second.segment.getJoint());
        links.back().jointValue = &jointPositions[4];
      }
      else if (links.back().name == "arm_link3")
      {
        links.back().joint = &((*child)->second.segment.getJoint());
        links.back().jointValue = &jointPositions[5];
      }
      else if (links.back().name == "arm_link4")
      {
        links.back().joint = &((*child)->second.segment.getJoint());
        links.back().jointValue = &jointPositions[6];
      }
      else if (links.back().name == "arm_link5")
      {
        links.back().joint = &((*child)->second.segment.getJoint());
        links.back().jointValue = &jointPositions[7];
      }
      else
        links.back().transformToParent = (*child)->second.segment.getFrameToTip();

      expandTreeKDL(*child, links.size() - 1);
    }
  }

  bool initializeFCL()
  {
    std::string robotDescription;
    nh.getParam("/squirrel_8dof_planning/robot_description", robotDescription);
    const boost::shared_ptr<urdf::ModelInterface> model = urdf::parseURDF(robotDescription);

    std::vector<boost::shared_ptr<urdf::Link> > links;
    model->getLinks(links);
    for (std::vector<boost::shared_ptr<urdf::Link> >::const_iterator link = links.begin(), end = links.end(); link != end; ++link)
    {
      if ((*link)->collision && (*link)->collision->geometry)
      {
        const boost::shared_ptr<urdf::Geometry> geometry = (*link)->collision->geometry;
        boost::shared_ptr<fcl::CollisionGeometry> cGeometry;

        if (geometry->type == urdf::Geometry::BOX)
        {
          const urdf::Box *box(static_cast<urdf::Box*>(&*geometry));
          if (box->dim.x > 0.0 && box->dim.y > 0.0 && box->dim.z > 0.0)
            cGeometry.reset(new fcl::Box(box->dim.x, box->dim.y, box->dim.z));
        }
        else if (geometry->type == urdf::Geometry::CYLINDER)
        {
          const urdf::Cylinder *cylinder(static_cast<urdf::Cylinder*>(&*geometry));
          if (cylinder->radius > 0.0 && cylinder->length > 0.0)
            cGeometry.reset(new fcl::Cylinder(cylinder->radius, cylinder->length));
        }
        else if (geometry->type == urdf::Geometry::SPHERE)
        {
          const urdf::Sphere *sphere(static_cast<urdf::Sphere*>(&*geometry));
          if (sphere->radius > 0.0)
            cGeometry.reset(new fcl::Sphere(sphere->radius));
        }
        else if (geometry->type == urdf::Geometry::MESH)
        {
          const urdf::Mesh *urdfMesh(static_cast<urdf::Mesh*>(&*geometry));
          const shapes::Mesh *mesh = shapes::createMeshFromResource(urdfMesh->filename,
                                                                    Eigen::Vector3d(urdfMesh->scale.x, urdfMesh->scale.y, urdfMesh->scale.z));
          fcl::BVHModel<fcl::OBBRSS> *model = new fcl::BVHModel<fcl::OBBRSS>();
          // code from moveit_core::collision_detection::createCollisionGeometry
          if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
          {
            std::vector<fcl::Triangle> tri_indices(mesh->triangle_count);
            for (unsigned int i = 0; i < mesh->triangle_count; ++i)
            {
              tri_indices[i] = fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);
            }

            std::vector<fcl::Vec3f> points(mesh->vertex_count);
            for (unsigned int i = 0; i < mesh->vertex_count; ++i)
            {
              points[i] = fcl::Vec3f(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);
            }

            model->beginModel();
            model->addSubModel(points, tri_indices);
            model->endModel();
            cGeometry.reset(model);
          }
        }
        if (cGeometry)
        {
          for (std::vector<Link>::iterator it = this->links.begin(); it != this->links.end(); ++it)
          {
            if ((*link)->name == it->name)
            {
              KDL::Frame frameAdjust;
              urdf::Pose &pose = (*link)->collision->origin;
              frameAdjust.p.data[0] = pose.position.x;
              frameAdjust.p.data[1] = pose.position.y;
              frameAdjust.p.data[2] = pose.position.z;
              frameAdjust.M = KDL::Rotation::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);

              it->transformToParent = it->transformToParent * frameAdjust;

              it->collisionObject = new fcl::CollisionObject(cGeometry);
              if (cGeometry->getNodeType() == fcl::NODE_TYPE::BV_OBBRSS)
              {
                const urdf::Mesh *urdfMesh(static_cast<urdf::Mesh*>(&*geometry));
                it->mesh = urdfMesh->filename;
              }
              break;
            }
          }
        }
      }
    }
    return true;
  }

  void findSelfCollisionPairs()
  {
    std::string pathSRDF;
    pathSRDF = ros::package::getPath("squirrel_8dof_planner");
    pathSRDF += "/config/robotino_plan.srdf";

    std::vector<std::pair<std::string, std::string> > disPairs;
    if (!parseSRDF(pathSRDF, disPairs) || disPairs.size() == 0 || links.size() == 0)
    {
      //all self collisions are used
      for (UInt i = 0; i < links.size(); ++i)
        for (UInt j = i + 1; j < links.size(); ++j)
        {
          if (links[i].collisionObject != NULL && links[j].collisionObject != NULL)
            selfCollisionPairs.push_back(std::make_pair(&links[i], &links[j]));
        }
    }
    else
    {
      //use disabled links to only add the other links
      std::map<std::string, UInt> linkIndices;
      for (UInt i = 0; i < links.size(); ++i)
        linkIndices[links[i].name] = i;

      std::vector<std::vector<bool> > collisionMatrix(links.size(), std::vector<bool>(links.size(), true));

      for (UInt i = 0; i < disPairs.size(); ++i)
      {
        collisionMatrix[linkIndices[disPairs[i].first]][linkIndices[disPairs[i].second]] = false;
        collisionMatrix[linkIndices[disPairs[i].second]][linkIndices[disPairs[i].first]] = false;
      }

      for (UInt i = 0; i < links.size(); ++i)
        for (UInt j = i + 1; j < links.size(); ++j)
          if (links[i].collisionObject != NULL && links[j].collisionObject != NULL && collisionMatrix[i][j])
            selfCollisionPairs.push_back(std::make_pair(&links[i], &links[j]));
    }

    std::cout << "Checking a total of " << selfCollisionPairs.size() << " self collision pairs and " << octomapCollisionLinks.size()
        << " link collisions with the octomap." << std::endl;
  }

  bool parseSRDF(const std::string &filePath, std::vector<std::pair<std::string, std::string> > &disabledPairs)
  {
    std::ifstream file(filePath.c_str());
    if (!file.good())
      return false;

    std::string line;
    while (std::getline(file, line))
    {
      if (line.find("<disable_collisions") != std::string::npos)
      {
        std::string link1, link2;
        getLinksFromString(line, link1, link2);
        if (link1 != "octomap" && link2 != "octomap")
          disabledPairs.push_back(std::make_pair(link1, link2));
        else
        {
          if (link1 == "octomap" && link2 == "octomap")
            continue;

          octomapCollisionLinks.erase(link1 == "octomap" ? link2 : link1);
        }
      }
    }

    return true;
  }

  void getLinksFromString(const std::string &line, std::string &link1, std::string &link2) const
  {
    UInt posStart = line.find("link1"), nameLength, posTmp;
    posStart += 7;
    posTmp = posStart;
    nameLength = 0;
    while (posTmp < line.size() && line.at(posTmp) != '"')
    {
      ++nameLength;
      ++posTmp;
    }

    link1 = line.substr(posStart, nameLength);

    posStart = line.find("link2");
    posStart += 7;
    posTmp = posStart;
    nameLength = 0;
    while (posTmp < line.size() && line.at(posTmp) != '"')
    {
      ++nameLength;
      ++posTmp;
    }

    link2 = line.substr(posStart, nameLength);
  }

  void publishCollisionModell()
  {
    if (publisherCollisionModell.getNumSubscribers() == 0)
      return;

    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = 500;
    for (std::vector<Link>::const_iterator it = links.begin(); it != links.end(); ++it)
    {
      const Link &link = *it;
      if (link.collisionObject == nullptr)
        continue;

      marker.pose.position.x = link.transform.p.data[0];
      marker.pose.position.y = link.transform.p.data[1];
      marker.pose.position.z = link.transform.p.data[2];
      link.transform.M.GetQuaternion(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w);
      marker.color.a = 0.3;
      marker.color.b = 0.2;
      marker.color.r = link.isInCollision ? 0.8 : 0.2;
      marker.color.g = link.isInCollision ? 0.2 : 0.8;

      switch (link.collisionObject->getNodeType())
      {
        case fcl::NODE_TYPE::BV_OBBRSS:
        {
          marker.type = visualization_msgs::Marker::MESH_RESOURCE;
          boost::shared_ptr<const fcl::Box> obj(boost::static_pointer_cast<const fcl::Box>(link.collisionObject->collisionGeometry()));
          marker.mesh_resource = link.mesh;
          marker.scale.x = 1.0;
          marker.scale.y = 1.0;
          marker.scale.z = 1.0;
        }
          break;
        case fcl::NODE_TYPE::GEOM_BOX:
        {
          marker.type = visualization_msgs::Marker::CUBE;
          boost::shared_ptr<const fcl::Box> obj(boost::static_pointer_cast<const fcl::Box>(link.collisionObject->collisionGeometry()));
          marker.scale.x = obj->side[0];
          marker.scale.y = obj->side[1];
          marker.scale.z = obj->side[2];
        }
          break;
        case fcl::NODE_TYPE::GEOM_CYLINDER:
        {
          marker.type = visualization_msgs::Marker::CYLINDER;
          boost::shared_ptr<const fcl::Cylinder> obj(boost::static_pointer_cast<const fcl::Cylinder>(link.collisionObject->collisionGeometry()));
          marker.scale.x = 2.0 * obj->radius;
          marker.scale.y = marker.scale.x;
          marker.scale.z = obj->lz;
        }
          break;
        default:
          continue;
      }
      msg.markers.push_back(marker);

      ++marker.id;
    }

    publisherCollisionModell.publish(msg);
  }

  // ******************** COLLISION CHECKING ********************

  void updateTransforms(const std::vector<Real> &jointPositions)
  {
    for (UInt i = 0; i < this->jointPositions.size(); ++i)
      this->jointPositions[i] = jointPositions[i];

    Real quatX, quatY, quatZ, quatW;
    for (std::vector<Link>::iterator link = links.begin() + 1; link != links.end(); ++link)
    {
      if (link->joint != NULL)
        link->transform = (*(link->transformParent)) * link->joint->pose(*(link->jointValue));
      else
        link->transform = (*(link->transformParent)) * link->transformToParent;

      if (link->collisionObject)
      {
        link->transform.M.GetQuaternion(quatX, quatY, quatZ, quatW);
        link->collisionObject->setTransform(fcl::Quaternion3f(quatW, quatX, quatY, quatZ),
                                            fcl::Vec3f(link->transform.p[0], link->transform.p[1], link->transform.p[2]));
      }
    }
  }

  bool checkSelfCollision()
  {
    for (std::vector<std::pair<Link*, Link*> >::const_iterator it = selfCollisionPairs.begin(); it != selfCollisionPairs.end(); ++it)
    {
      fcl::CollisionRequest request;
      fcl::CollisionResult result;
      fcl::collide(it->first->collisionObject, it->second->collisionObject, request, result);
      if (result.isCollision())
        return true;
    }
    return false;
  }

  bool checkMapCollision()
  {
    if (!octomapCollisionObject)
      return false;

    if (checkAllOcotmapLinkCollisions)
    {
      for (std::vector<Link>::const_iterator it = links.begin(); it != links.end(); ++it)
      {
        if (!it->collisionObject)
          continue;
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        fcl::collide(octomapCollisionObject, it->collisionObject, request, result);
        if (result.isCollision())
          return true;
      }
      return false;
    }
    else
    {
      for (std::map<std::string, std::pair<Link*, bool> >::const_iterator it = octomapCollisionLinks.begin(); it != octomapCollisionLinks.end(); ++it)
      {
        if (!it->second.second)
          continue;

        const fcl::CollisionObject* collisionObject = it->second.first->collisionObject;

        if (!collisionObject)
          continue;
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        fcl::collide(octomapCollisionObject, collisionObject, request, result);
        if (result.isCollision())
          return true;
      }
      return false;
    }
  }

  void getSelfCollisions(std::vector<std::pair<std::string, std::string> > &selfCollisions)
  {
    for (std::vector<std::pair<Link*, Link*> >::const_iterator it = selfCollisionPairs.begin(); it != selfCollisionPairs.end(); ++it)
    {
      fcl::CollisionRequest request;
      fcl::CollisionResult result;
      fcl::collide(it->first->collisionObject, it->second->collisionObject, request, result);
      if (result.isCollision())
      {
        it->first->isInCollision = true;
        it->second->isInCollision = true;
        selfCollisions.push_back(std::make_pair(it->first->name, it->second->name));
      }
    }
  }

  void getMapCollisions(std::vector<std::string> &mapCollisions)
  {
    if (!octomapCollisionObject)
      return;

    for (std::map<std::string, std::pair<Link*, bool> >::const_iterator it = octomapCollisionLinks.begin(); it != octomapCollisionLinks.end(); ++it)
    {
      const fcl::CollisionObject* collisionObject = it->second.first->collisionObject;

      if (!collisionObject)
        continue;
      fcl::CollisionRequest request;
      fcl::CollisionResult result;
      fcl::collide(octomapCollisionObject, collisionObject, request, result);
      if (result.isCollision())
      {
        it->second.first->isInCollision = true;
        mapCollisions.push_back(it->first);
      }
    }
  }
};

} // namespace birrt_star_motion_planning

#endif // BIRRT_STAR_ALGORITHM_COLLISION_CHECKER_HPP_

