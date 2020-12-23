//
// Created by jieming on 11.12.20.
//

#include "CollisionCheck.h"
//public
CollisionCheck::CollisionCheck() : initialized(false), octomapCollisionObject(nullptr),
        manager1(nullptr), manager2(nullptr), isFirstReceive(true), octTree_(0.05) {
    if (!initialize()){
        ros::shutdown();
        return;
    }

    std::string octomapName;
    nh.getParam("/map_file", octomapName);
    octTree_.readBinary(octomapName);

    setOcTree(&octTree_);
    octreePtr_.reset(new fcl::OcTree<double>( static_cast<const std::shared_ptr<const octomap::OcTree>>(&octTree_) ));

    fcl::test::generateBoxesFromOctomap(map_boxes, *octreePtr_, box_sizes);


    publisherCollisionModell = nh.advertise<visualization_msgs::MarkerArray>("collision_modell", 3);
    publisherNearObs = nh.advertise<visualization_msgs::MarkerArray>("near_obs", 2);
    publisherNearBoxes = nh.advertise<std_msgs::Float32MultiArray>("near_boxes", 2);
    sensorSub = nh.subscribe("/robot1/joint_states", 1, &CollisionCheck::jointStatesCallback, this);
//    traj_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",1);
    cout << "construction function finished jm" << endl;
    manager1.reset(new fcl::DynamicAABBTreeCollisionManagerd());
    manager1->clear();
    for(const auto& link: links){
        if(!link.mesh.empty() && link.name!="panda_link0" ){
             manager1->registerObject(link.collisionObject.get());
        }
    }
    manager1->setup();
    //manager2.reset(new fcl::DynamicAABBTreeCollisionManagerd());
//    manager2->registerObject(octomapCollisionObject.get());
//    manager2->setup();  //octomap



    initialized = true;
}

void CollisionCheck::publishCollisionModell(){
    if (publisherCollisionModell.getNumSubscribers() == 0)
        return;

    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;
    msg.markers.reserve(8);

    marker.header.frame_id = "panda1/world";
    marker.header.stamp = ros::Time::now();
    marker.id = 500;
    for (std::vector<Link>::const_iterator it = links.begin(); it != links.end(); ++it){
        const Link &link = *it;
        if (link.collisionObject == nullptr){
            continue;
        }

        marker.pose.position.x = link.transform.p.data[0];
        marker.pose.position.y = link.transform.p.data[1];
        marker.pose.position.z = link.transform.p.data[2];

        link.transform.M.GetQuaternion(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w);
        marker.color.a = 1;
        marker.color.b = 0.2;
        marker.color.r = link.isInCollision ? 0.8 : 0.2;
        marker.color.g = link.isInCollision ? 0.2 : 0.8;

        switch (link.collisionObject->getNodeType()){
            case fcl::NODE_TYPE::BV_OBBRSS:{
                marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                std::shared_ptr<const fcl::Boxd> obj(std::static_pointer_cast<const fcl::Boxd>(link.collisionObject->collisionGeometry()));
                marker.mesh_resource = link.mesh;
                marker.scale.x = 1.0;
                marker.scale.y = 1.0;
                marker.scale.z = 1.0;
            }
                break;
            case fcl::NODE_TYPE::GEOM_BOX:{
                marker.type = visualization_msgs::Marker::CUBE;
                std::shared_ptr<const fcl::Boxd> obj(std::static_pointer_cast<const fcl::Boxd>(link.collisionObject->collisionGeometry()));
                marker.scale.x = obj->side[0];
                marker.scale.y = obj->side[1];
                marker.scale.z = obj->side[2];
            }
                break;
            case fcl::NODE_TYPE::GEOM_CYLINDER:{
                marker.type = visualization_msgs::Marker::CYLINDER;
                std::shared_ptr<const fcl::Cylinderd> obj(std::static_pointer_cast<const fcl::Cylinderd>(link.collisionObject->collisionGeometry()));
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
    msg.markers.shrink_to_fit();
    publisherCollisionModell.publish(msg);
}

void CollisionCheck::showBoxes() {
    std::vector<Eigen::Vector3d> near_points;
    near_points.reserve(3000);

    for (auto &box: map_boxes) {
        near_points.push_back(box->getTranslation());
    }
    near_points.shrink_to_fit();
    publishInRange(near_points, box_sizes);
}  //map voxel

void CollisionCheck::publishInRange(const std::vector<Eigen::Vector3d>& near_points, std::vector<double>& sizes){
    if (publisherNearObs.getNumSubscribers() == 0)
        return;
    if(map_boxes.empty()){  // redundan?
        return;
    }
    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;
    msg.markers.reserve(5000);

    static uint last_id=0;
    marker.header.frame_id = "panda1/world";
    marker.header.stamp = ros::Time::now();
    marker.id = 600;

    for (std::size_t i=0; i<near_points.size(); ++i){
        const auto near_point = near_points[i];

        marker.pose.position.x = near_point(0);
        marker.pose.position.y = near_point(1);
        marker.pose.position.z = near_point(2);

        marker.color.a = 1;
        marker.color.b = 0.35;
        marker.color.r = 0.85;
        marker.color.g = 0.81;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::MODIFY;
//        cout << map_boxes[0]->collisionGeometry()->aabb_radius << endl;
        marker.scale.x = sizes[i] *1.1;
        marker.scale.y = sizes[i] *1.1;
        marker.scale.z = sizes[i]*1.1;

        marker.pose.orientation.w = 1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;

        msg.markers.push_back(marker);
        ++marker.id;
    }
    int cur_id = marker.id-1;

    for(std::size_t i=cur_id+1; i<=last_id; i++){
        marker.action = visualization_msgs::Marker::DELETE;
        marker.id = i;
        msg.markers.push_back(marker);
    }

    msg.markers.shrink_to_fit();
    publisherNearObs.publish(msg);
    last_id = cur_id;
}

void CollisionCheck::publishBoxes(const std::vector<Eigen::Vector3d>& near_points, std::vector<double>& sizes){
    if (publisherNearBoxes.getNumSubscribers() == 0)
        return;

    std_msgs::Float32MultiArray msg;
    msg.data.reserve(2000);

    if(map_boxes.empty()){
       //publish zero sizes
       msg.data.push_back(0);
    }
    else{
        msg.data.push_back(near_points.size());

        for (std::size_t i=0; i<near_points.size(); ++i){
            const auto near_point = near_points[i];
            msg.data.push_back(near_point(0));
            msg.data.push_back(near_point(1));
            msg.data.push_back(near_point(2));
            msg.data.push_back(sizes[i]);
        }
    }
    msg.data.shrink_to_fit();
    publisherNearBoxes.publish(msg);
}


bool CollisionCheck::isFirstReceiv(){
    return isFirstReceive;
}

void CollisionCheck::setOcTree(const octomap::OcTree* octree){
    if (octree == nullptr)
        return;

    const std::shared_ptr<const octomap::OcTree> octreePtr = std::make_shared<const octomap::OcTree>(*octree);
    std::shared_ptr<fcl::CollisionGeometryd> octomapCollisionGeometry(new fcl::OcTreed(octreePtr));

    octomapCollisionObject.reset(new fcl::CollisionObjectd(octomapCollisionGeometry));
    octomapCollisionObject->setTransform(fcl::Quaterniond(1, 0, 0, 0), fcl::Vector3d (0, 0, -0.02));  //TODO ???? -0.02
}

void CollisionCheck::setDisabledLinkMapCollisions(const std::vector<std::string> &linksDisabled){
    if (linksDisabled.size() == 0 && octomapCollisionLinks.size() == links.size())
        checkAllOcotmapLinkCollisions = true;
    else{
        checkAllOcotmapLinkCollisions = false;
        for (std::map<std::string, std::pair<Link*, bool> >::iterator it = octomapCollisionLinks.begin(); it != octomapCollisionLinks.end(); ++it)
            it->second.second = true;
        for (std::vector<std::string>::const_iterator it = linksDisabled.begin(); it != linksDisabled.end(); ++it)
            octomapCollisionLinks[*it].second = false;
    }
}

bool CollisionCheck::isInCollision(const std::vector<Real> &joint_positions,
                    const bool checkSelfCollision, const bool checkMapCollision){
    if (!initialized)
        return true;

    if (!checkSelfCollision && !checkMapCollision)
        return false;

    updateTransforms(joint_positions);

    if (checkMapCollision && this->checkMapCollision())
        return true;

    if (checkSelfCollision && this->checkSelfCollision())
        return true;

    return false;
}

void CollisionCheck::getCollisions(const std::vector<Real> &joint_positions, std::vector<std::pair<std::string, std::string> > &selfCollisions,
                   std::vector<std::string> &mapCollisions){

    for (std::vector<Link>::iterator it = links.begin(); it != links.end(); ++it)
        it->isInCollision = false;

    getSelfCollisions(selfCollisions);
    getMapCollisions(mapCollisions);
    publishCollisionModell();
}

void CollisionCheck::getCollisions(){
    std::vector<std::pair<std::string, std::string> > selfCollisions;
    std::vector<std::string> mapCollisions;


    for (std::vector<Link>::iterator it = links.begin(); it != links.end(); ++it)
        it->isInCollision = false;

    getSelfCollisions(selfCollisions);
    getMapCollisions(mapCollisions);
    publishCollisionModell();


}

void CollisionCheck::getCollisionsInRange(const double& range) {

    fcl::DistanceRequestd request;
//    request.enable_signed_distance = true;
//    request.enable_nearest_points = true;
    request.gjk_solver_type = fcl::GST_LIBCCD;
//    request.gjk_solver_type = fcl::GST_INDEP;
    fcl::DistanceResultd result;

    double dist(1000);
    std::vector<Eigen::Vector3d> near_points;
    std::vector<double> sizes;
    near_points.reserve(3000);
    sizes.reserve(3000);
    int cnt=0;
    for (auto &box: map_boxes) {
        for (auto it = octomapCollisionLinks.rbegin(); it != octomapCollisionLinks.rend(); it++) {
            if (!it->second.second) {
                continue;
            }
            fcl::CollisionObjectd *collisionObject = it->second.first->collisionObject.get();

            result.clear();
            distance(collisionObject, box, request, result);
            dist = result.min_distance;
            if (dist <= range) {
                near_points.push_back(box->getTranslation());
                sizes.push_back(box_sizes[cnt]);
                break;
            }
        }
        cnt ++;
    }
    near_points.shrink_to_fit();
    sizes.shrink_to_fit();
    publishInRange(near_points, sizes);
    publishBoxes(near_points, sizes);

}


void CollisionCheck::test_distance_spheresphere()
{
    typedef  double S ;
    using namespace fcl;
    const S radius_1 = 0.2;
    const S radius_2 = 0.2;
    fcl::Sphere<S> s1{radius_1};
    fcl::Sphere<S> s2{radius_2};

    fcl::Transform3<S> tf1{Transform3<S>::Identity()};
    Transform3<S> tf2{Transform3<S>::Identity()};

    DistanceRequest<S> request;
    request.enable_signed_distance = true;
    request.enable_nearest_points = true;
    request.gjk_solver_type = GST_LIBCCD;
    DistanceResult<S> result;

    result.clear();
    tf2.translation() = Vector3<S>(3, 4, 5);

    fcl::Box<S> s3{radius_1,radius_1,radius_1};

    distance(&s3, tf1, &s3, tf2, request, result);
    cout << result.min_distance << endl;
}

// ******************** collision check server ********************
bool CollisionCheck::checkCollision(const std::vector<Real> &jointPositions1){
    for (std::vector<Link>::iterator it = links.begin(); it != links.end(); ++it)
        it->isInCollision = false;

    updateTransforms(jointPositions1);
    bool collide1 = checkMapCollision();
    bool collide2 = checkSelfCollision();
//    return collide1 || collide2;
    return collide2;
}



//private
void CollisionCheck::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg){
    if(!callJointStateCB) 
	return;
    if(isFirstReceive){
        isFirstReceive = false;
        cout << "first receive joint states"  << endl;
    }
    updateTransforms(msg->position);
}

bool CollisionCheck::initialize(){
    if (!initializeKDL())
        return false;

    if (!initializeFCL())
        return false;

    checkAllOcotmapLinkCollisions = false;
    findSelfCollisionPairs();

    return true;
}

bool CollisionCheck::initializeFCL(){
    std::string robotDescription("/home/jieming/catkin_ws/src/panda_simulation/franka_description/robots/model_hand.urdf");
    //const auto model = urdf::parseURDF(robotDescription);
    boost::shared_ptr<urdf::Model> model(new urdf::Model);
    if (!model->initFile(robotDescription)){
        cout << "Failed to parse urdf file" << endl;
    }
    std::vector<urdf::LinkSharedPtr> links;
    model->getLinks(links);

    for(auto link = links.begin(); link != links.end(); ++link){
        if ((*link)->collision && (*link)->collision->geometry){
            const urdf::GeometrySharedPtr geometry = (*link)->collision->geometry;
            std::shared_ptr<fcl::CollisionGeometryd> cGeometry;

            if (geometry->type == urdf::Geometry::BOX){
                const urdf::Box *box(static_cast<urdf::Box*>(&*geometry));
                if (box->dim.x > 0.0 && box->dim.y > 0.0 && box->dim.z > 0.0)
                    cGeometry.reset(new fcl::Boxd(box->dim.x, box->dim.y, box->dim.z));
            }
            else if (geometry->type == urdf::Geometry::CYLINDER){
                const urdf::Cylinder *cylinder(static_cast<urdf::Cylinder*>(&*geometry));
                if (cylinder->radius > 0.0 && cylinder->length > 0.0)
                    cGeometry.reset(new fcl::Cylinderd(cylinder->radius, cylinder->length));
            }
            else if (geometry->type == urdf::Geometry::SPHERE){
                const urdf::Sphere *sphere(static_cast<urdf::Sphere*>(&*geometry));
                if (sphere->radius > 0.0)
                    cGeometry.reset(new fcl::Sphered(sphere->radius));
            }
            else if (geometry->type == urdf::Geometry::MESH){
                const urdf::Mesh *urdfMesh(static_cast<urdf::Mesh*>(&*geometry));
                const shapes::Mesh *mesh = shapes::createMeshFromResource(urdfMesh->filename,
                                                                          Eigen::Vector3d(urdfMesh->scale.x, urdfMesh->scale.y, urdfMesh->scale.z));
                fcl::BVHModel<fcl::OBBRSSd> *model = new fcl::BVHModel<fcl::OBBRSSd>();
                // code from moveit_core::collision_detection::createCollisionGeometry
                if (mesh->vertex_count > 0 && mesh->triangle_count > 0){
                    std::vector<fcl::Triangle> tri_indices(mesh->triangle_count);
                    for (unsigned int i = 0; i < mesh->triangle_count; ++i){
                        tri_indices[i] = fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);
                    }

                    std::vector<fcl::Vector3d> points(mesh->vertex_count);
                    for (unsigned int i = 0; i < mesh->vertex_count; ++i){
                        points[i] = fcl::Vector3d(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);
                    }

                    model->beginModel();
                    model->addSubModel(points, tri_indices);
                    model->endModel();
                    cGeometry.reset(model);
                }
            }
            if (cGeometry){
                for (std::vector<Link>::iterator it = this->links.begin(); it != this->links.end(); ++it){
                    if ((*link)->name == it->name){
                        KDL::Frame frameAdjust;
                        urdf::Pose &pose = (*link)->collision->origin;
                        frameAdjust.p.data[0] = pose.position.x;
                        frameAdjust.p.data[1] = pose.position.y;
                        frameAdjust.p.data[2] = pose.position.z;
                        frameAdjust.M = KDL::Rotation::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);

                        it->transformToParent = it->transformToParent * frameAdjust;
                        it->collisionObject.reset(new fcl::CollisionObjectd(cGeometry));//it->collisionObject = new fcl::CollisionObjectd(cGeometry);

                        if (cGeometry->getNodeType() == fcl::NODE_TYPE::BV_OBBRSS){
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

void CollisionCheck::findSelfCollisionPairs(){
    std::string pathSRDF;
    pathSRDF = ros::package::getPath("franka_description");
    pathSRDF += "/config/panda.srdf";

    std::vector<std::pair<std::string, std::string> > disPairs;
    if (!parseSRDF(pathSRDF, disPairs) || disPairs.size() == 0 || links.size() == 0){
        //all self collisions are used
        for (UInt i = 0; i < links.size(); ++i)
            for (UInt j = i + 1; j < links.size(); ++j)
            {
                if (links[i].collisionObject != NULL && links[j].collisionObject != NULL)
                    selfCollisionPairs.push_back(std::make_pair(&links[i], &links[j]));
            }
    }
    else{
        //use disabled links to only add the other links
        std::map<std::string, UInt> linkIndices;
        for (UInt i = 0; i < links.size(); ++i)
            linkIndices[links[i].name] = i;

        std::vector<std::vector<bool> > collisionMatrix(links.size(), std::vector<bool>(links.size(), true));

        for (UInt i = 0; i < disPairs.size(); ++i){
            collisionMatrix[linkIndices[disPairs[i].first]][linkIndices[disPairs[i].second]] = false;
            collisionMatrix[linkIndices[disPairs[i].second]][linkIndices[disPairs[i].first]] = false;
        }

        for (UInt i = 0; i < links.size(); ++i)
            for (UInt j = i + 1; j < links.size(); ++j)
                if (links[i].collisionObject != NULL && links[j].collisionObject != NULL && collisionMatrix[i][j]){
                    selfCollisionPairs.push_back(std::make_pair(&links[i], &links[j]));
                    cout << links[i].name  << "  " << links[j].name << endl;
                }
    }
    std::cout << "Checking a total of " << selfCollisionPairs.size() << " self collision pairs" << endl;

    for(const auto& i: octomapCollisionLinks){
        cout << i.first << " " << i.second.second << endl;
    }
    cout <<"and " << octomapCollisionLinks.size() << " link collisions with the octomap." << std::endl;
}

bool CollisionCheck::parseSRDF(const std::string &filePath, std::vector<std::pair<std::string, std::string> > &disabledPairs){
    std::ifstream file(filePath.c_str());
    if (!file.good())
        return false;

    std::string line;
    while (std::getline(file, line)){
        if (line.find("<disable_collisions") != std::string::npos){
            std::string link1, link2;
            getLinksFromString(line, link1, link2);
            if (link1 != "octomap" && link2 != "octomap")
                disabledPairs.push_back(std::make_pair(link1, link2));
            else{
                if (link1 == "octomap" && link2 == "octomap")
                    continue;

                octomapCollisionLinks.erase(link1 == "octomap" ? link2 : link1);
            }
        }
    }

    return true;
}

void CollisionCheck::getLinksFromString(const std::string &line, std::string &link1, std::string &link2) const{
    UInt posStart = line.find("link1"), nameLength, posTmp;
    posStart += 7;
    posTmp = posStart;
    nameLength = 0;
    while (posTmp < line.size() && line.at(posTmp) != '"'){
        ++nameLength;
        ++posTmp;
    }

    link1 = line.substr(posStart, nameLength);

    posStart = line.find("link2");
    posStart += 7;
    posTmp = posStart;
    nameLength = 0;
    while (posTmp < line.size() && line.at(posTmp) != '"'){
        ++nameLength;
        ++posTmp;
    }

    link2 = line.substr(posStart, nameLength);
//        cout << link1 << endl;
//        cout << link2 << endl;
}


bool CollisionCheck::initializeKDL(){
//        bool readFine = kdl_parser::treeFromFile("/home/jieming/catkin_ws/src/panda_simulation/franka_description/robots/model_hand.urdf", kdlTree);
    bool readFine = kdl_parser::treeFromParam("/robot1/robot_description", kdlTree);

    if (!readFine){
        ROS_ERROR("Could not parse urdf.");
        return false;
    }

    jointPositions.resize(9, 0.0);
    createLinksKDL();

    for(auto it = links.begin(); it != links.end(); ++it){
        octomapCollisionLinks[it->name] = std::make_pair(&(*it), true);
        cout << it->name << endl;
    }
    std::cout << "Loaded " << links.size() << " links into the collision modell (include world panda_link0)." << std::endl;

    return true;
}

void CollisionCheck::createLinksKDL(){
    links.reserve(200);
    links.emplace_back();

    links.back().name = kdlTree.getRootSegment()->second.segment.getName();
    links.back().transform = KDL::Frame::Identity();
    links.back().transform.p.data[2] = 0;
    links.back().transformToParent = KDL::Frame::Identity();
    expandTreeKDL(kdlTree.getRootSegment(), 0);

}

void CollisionCheck::expandTreeKDL(const KDL::SegmentMap::const_iterator& segment, UInt indexParent){
    for(auto child = segment->second.children.begin(); child != segment->second.children.end();  ++child){

        links.push_back(Link());
        links.back().name = (*child)->second.segment.getName();

        links.back().transformParent = &links[indexParent].transform;

        if (links.back().name == "panda_link1"){
            links.back().joint = &((*child)->second.segment.getJoint());
            links.back().jointValue = &jointPositions[0];
            links.back().segment = &((*child)->second.segment);
        }
        else if (links.back().name == "panda_link2"){
            links.back().joint = &((*child)->second.segment.getJoint());
            links.back().jointValue = &jointPositions[1];
            links.back().segment = &((*child)->second.segment);
        }
        else if (links.back().name == "panda_link3"){
            links.back().joint = &((*child)->second.segment.getJoint());
            links.back().jointValue = &jointPositions[2];
            links.back().segment = &((*child)->second.segment);
        }
        else if (links.back().name == "panda_link4"){
            links.back().joint = &((*child)->second.segment.getJoint());
            links.back().jointValue = &jointPositions[3];
            links.back().segment = &((*child)->second.segment);
        }
        else if (links.back().name == "panda_link5"){
            links.back().joint = &((*child)->second.segment.getJoint());
            links.back().jointValue = &jointPositions[4];
            links.back().segment = &((*child)->second.segment);
        }
        else if (links.back().name == "panda_link6"){
            links.back().joint = &((*child)->second.segment.getJoint());
            links.back().jointValue = &jointPositions[5];
            links.back().segment = &((*child)->second.segment);
        }
        else if (links.back().name == "panda_link7"){
            links.back().joint = &((*child)->second.segment.getJoint());
            links.back().jointValue = &jointPositions[6];
            links.back().segment = &((*child)->second.segment);
        }
        else if (links.back().name == "panda_leftfinger"){
            links.back().joint = &((*child)->second.segment.getJoint());
            links.back().jointValue = &jointPositions[7];
            links.back().segment = &((*child)->second.segment);
        }
        else if (links.back().name == "panda_rightfinger"){
            links.back().joint = &((*child)->second.segment.getJoint());
            links.back().jointValue = &jointPositions[8];
            links.back().segment = &((*child)->second.segment);
        }
        else  // for no joint situation
            links.back().transformToParent = (*child)->second.segment.getFrameToTip();

        expandTreeKDL(*child, links.size() - 1);
    }
}


// ******************** COLLISION CHECKING ********************
void CollisionCheck::updateTransforms(const std::vector<Real> &jointPositions1){
    for (UInt i = 0; i < this->jointPositions.size(); ++i){
        this->jointPositions[i] = jointPositions1[i+2];
//        if(i>6)
//            this->jointPositions[i] = 0; // temp TODO JM for grasp
    }
    Real quatX, quatY, quatZ, quatW;

    for (std::vector<Link>::iterator link = links.begin() + 1; link != links.end(); ++link){
        if (link->joint != nullptr){
//                link->transform = (*(link->transformParent)) * link->joint->pose(*(link->jointValue));
//                link->transform = (*(link->transformParent)) *  kdlTree.getSegment(link->name)->second.segment.pose(*(link->jointValue));
            link->transform = (*(link->transformParent)) *  link->segment->pose(*(link->jointValue));
        }
        else
            link->transform = (*(link->transformParent)) * link->transformToParent;

        if (link->collisionObject){
            link->transform.M.GetQuaternion(quatX, quatY, quatZ, quatW);
            link->collisionObject->setTransform(fcl::Quaterniond(quatW, quatX, quatY, quatZ),
                                                fcl::Vector3d (link->transform.p[0], link->transform.p[1], link->transform.p[2]));
        }
    }
}

bool CollisionCheck::checkSelfCollision(){
    bool collision_flag = false;
    for (std::vector<std::pair<Link*, Link*> >::const_iterator it = selfCollisionPairs.begin(); it != selfCollisionPairs.end(); ++it){
        fcl::CollisionRequestd request;
        fcl::CollisionResultd result;
        fcl::collide(it->first->collisionObject.get(), it->second->collisionObject.get(), request, result);
        if (result.isCollision()){
            collision_flag = true;
            break;
        }
    }
    return collision_flag;
}

void CollisionCheck::getSelfCollisions(std::vector<std::pair<std::string, std::string> > &selfCollisions){

    for (std::vector<std::pair<Link*, Link*> >::const_iterator it = selfCollisionPairs.begin(); it != selfCollisionPairs.end(); ++it){
        fcl::CollisionRequestd request;
        fcl::CollisionResultd result;
        fcl::collide(it->first->collisionObject.get(), it->second->collisionObject.get(), request, result);
        if (result.isCollision()){
            it->first->isInCollision = true;
            it->second->isInCollision = true;
//            selfCollisions.push_back(std::make_pair(it->first->name, it->second->name));
        }
    }
}

bool CollisionCheck::checkMapCollision(){
    if (!octomapCollisionObject)
        return false;

    if (checkAllOcotmapLinkCollisions){
        for (std::vector<Link>::const_iterator it = links.begin(); it != links.end(); ++it){
            if (!it->collisionObject)
                continue;

            fcl::CollisionRequestd request;
            fcl::CollisionResultd result;
            fcl::collide(octomapCollisionObject.get(), it->collisionObject.get(), request, result);
            if (result.isCollision())
                return true;
        }
        return false;
    }
    else{
        for (std::map<std::string, std::pair<Link*, bool> >::const_iterator it = octomapCollisionLinks.begin(); it != octomapCollisionLinks.end(); ++it){
            if (!it->second.second){  // bool flag false
                continue;
            }
            const fcl::CollisionObjectd* collisionObject = it->second.first->collisionObject.get();
            if (!collisionObject){
                continue;
            }
            fcl::CollisionRequestd request;
            fcl::CollisionResultd result;
            fcl::collide(octomapCollisionObject.get(), collisionObject, request, result);
            if (result.isCollision()){
                return true;
            }
        }
        return false;
    }
}

void CollisionCheck::getMapCollisions(std::vector<std::string> &mapCollisions){
    if (!octomapCollisionObject)
        return;

    for (std::map<std::string, std::pair<Link*, bool> >::const_iterator it = octomapCollisionLinks.begin(); it != octomapCollisionLinks.end(); ++it){
        const fcl::CollisionObjectd* collisionObject = it->second.first->collisionObject.get();

        if (!collisionObject)
            continue;
        fcl::CollisionRequestd request;
        fcl::CollisionResultd result;
        fcl::collide(octomapCollisionObject.get(), collisionObject, request, result);
        if (result.isCollision()){
            it->second.first->isInCollision = true;
//            mapCollisions.push_back(it->first);
        }
    }
}

CollisionCheck::~CollisionCheck(){
    for(std::size_t j = 0; j < map_boxes.size(); ++j)
        delete map_boxes[j];
    delete box2;

}

//void CollisionCheck::getCollisionsInRangeManager(const double& range) {
//
//    fcl::DefaultDistanceData<double> distance_data;
//    distance_data.request.gjk_solver_type = fcl::GST_LIBCCD;
//    distance_data.request.enable_nearest_points = false;
//    distance_data.request.enable_nearest_points = false;
//
//    double dist(1000);
//    std::vector<Eigen::Vector3d> near_points;
//
//    for (auto &box: map_boxes) {
//        distance_data.result.clear();
//        distance_data.done = false;
//        manager1->distance(box, &distance_data, DistanceFunction);
//        dist = distance_data.result.min_distance;
//        if (dist <= range) {
//            near_points.push_back(box->getTranslation());
//            break;
//        }
//    }
//
//    publishInRange(near_points);
//
//}
