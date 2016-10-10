//
//Copyright (c) 2016, Nikhil Somani
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//* Redistributions of source code must retain the above copyright notice,
//  this list of conditions and the following disclaimer.
//* Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.
//


#include "ConstraintParser.h"

inline Eigen::VectorXd readJsonVector(Json::Value Array){
    int size = Array.size();

    Eigen::VectorXd Vector(size);
    for(int i = 0 ; i < size ; ++i){
        Vector(i) = Array[i].asDouble();
    }
    return Vector;
}

inline Eigen::MatrixXd readJsonMatrix(Json::Value Array){
    int rows = Array.size();
    int cols = Array[0].size();

    Eigen::MatrixXd Matrix(rows,cols) ;

    for(int i = 0 ; i<rows ; ++i){
        for(int j=0 ; j<cols ; ++j){
            Matrix(i,j) = Array[i][j].asDouble() ;
        }
    }
    return Matrix;
}

ConstraintParser::ConstraintParser()
{
}

int ConstraintParser::addObjectModel(std::string rigid_object_name)
{
    for(std::size_t ro_id = 0;ro_id < rigid_objects.size();++ro_id)
    {
        if(rigid_objects[ro_id]->name == rigid_object_name)
            return ro_id;
    }
    ObjectModel *ro = new ObjectModel(rigid_object_name);
    rigid_objects.push_back(ro);
    std::pair<int, std::vector<int> > dependency;
    dependency.first = -1;
    dependencies.push_back(dependency);
    return (rigid_objects.size()-1);
}

void ConstraintParser::addDependency(int base_object_id, int dependent_object_id)
{
    bool redundant = false;
    for(std::size_t dependency_id = 0;dependency_id < dependencies[base_object_id].second.size();++dependency_id)
    {
        if(dependencies[base_object_id].second[dependency_id] == dependent_object_id)
        {
            redundant = true;
            break;
        }
    }
    if(!redundant)
        dependencies[base_object_id].second.push_back(dependent_object_id);
}

void ConstraintParser::addGeometricEntitiy(int rigid_object_id, PrimitiveShapeDescriptor* ps)
{
    rigid_objects[rigid_object_id]->primitive_shapes.push_back(ps);
}

ObjectModel *ConstraintParser::parse_constraint_json(std::string constraints_json, std::vector<Constraint*> &constraints_vec, std::stringstream &errorMsg, std::vector<Eigen::Matrix4d> &current_pose, std::string &solver_type)
{
    int constrained_object_id = -1;
    Json::Value root;
    Json::Reader reader;
    bool read_success = reader.parse(constraints_json, root, false);
    if(!read_success)
    {
        std::cout << "file couldn't be read" << std::endl;
        return nullptr;
    }

    rigid_objects.clear();
    Json::Value object_poses = root["objectPoses"];
    for(int object_id = 0;object_id < object_poses.size();++object_id)
    {
        Json::Value object = object_poses[object_id];
        int rigid_object_id = addObjectModel(std::string(object["name"].asString()));
        Eigen::Matrix4d T_object = readJsonMatrix(object["transformation"]);
        rigid_objects[rigid_object_id]->transform(T_object);
        current_pose.push_back(T_object);
    }

    bool success = true;

    solver_type = root["solver"].asString();    

    if(!root.isMember("constraints"))
    {
        errorMsg << "no constraints";
        return nullptr;
    }

    Json::Value constraints = root["constraints"];
    int num_constraints = constraints.size();
    for(int constraint_id = 0;constraint_id < num_constraints;++constraint_id)
    {
        Json::Value constraint = constraints[constraint_id];

        int rigid_object_fixed_id = addObjectModel(std::string(rigid_objects[constraint["fixed"]["poseIndex"].asInt()]->name));
        int rigid_object_constrained_id = addObjectModel(std::string(rigid_objects[constraint["constrained"]["poseIndex"].asInt()]->name));
        constrained_object_id = rigid_object_constrained_id;
        addDependency(rigid_object_constrained_id, rigid_object_fixed_id);

        std::string constraint_geoms = constraint["geometries"].asString();
        std::string constraint_type = constraint["type"].asString();
        if(constraint_geoms == "point_point")
        {
            PointShapeDescriptor *pt_constrained = new PointShapeDescriptor(readJsonVector(constraint["constrained"]["point"]));
            addGeometricEntitiy(rigid_object_constrained_id, pt_constrained);
            PointShapeDescriptor *pt_fixed = new PointShapeDescriptor(readJsonVector(constraint["fixed"]["point"]));
            addGeometricEntitiy(rigid_object_fixed_id, pt_fixed);
            if(constraint_type == "coincident")
            {
                constraints_vec.push_back(new PointPointCoincidentConstraint(pt_fixed, pt_constrained));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "distance")
            {
                constraints_vec.push_back(new PointPointDistanceConstraint(pt_fixed, pt_constrained, constraint["distance"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "distance_min_max")
            {
                constraints_vec.push_back(new PointPointDistanceMinMaxConstraint(pt_fixed, pt_constrained, constraint["distance_min"].asDouble(), constraint["distance_max"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else
            {
                errorMsg << "Constraint " << constraint_type << " not implemented for " << constraint_geoms;
                success = false;
                break;
            }
        }
        else if(constraint_geoms == "plane_plane")
        {
            bool flip = constraint["flip"].asBool();
            PlaneShapeDescriptor *pl_constrained = new PlaneShapeDescriptor(readJsonVector(constraint["constrained"]["point"]),
                                                                 readJsonVector(constraint["constrained"]["normal"]));
            addGeometricEntitiy(rigid_object_constrained_id, pl_constrained);
            PlaneShapeDescriptor *pl_fixed = new PlaneShapeDescriptor(readJsonVector(constraint["fixed"]["point"]).transpose(),
                                                                 ((flip)? -1:1)*readJsonVector(constraint["fixed"]["normal"]));
            addGeometricEntitiy(rigid_object_fixed_id, pl_fixed);
            if(constraint_type == "coincident")
            {
                constraints_vec.push_back(new PlanePlaneCoincidentConstraint(pl_fixed, pl_constrained));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "distance")
            {
                constraints_vec.push_back(new PlanePlaneDistanceConstraint(pl_fixed, pl_constrained, constraint["distance"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "distance_min_max")
            {
                constraints_vec.push_back(new PlanePlaneDistanceMinMaxConstraint(pl_fixed, pl_constrained, constraint["distance_min"].asDouble(), constraint["distance_max"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "parallel")
            {
                constraints_vec.push_back(new PlanePlaneParallelConstraint(pl_fixed, pl_constrained));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "angle")
            {
                constraints_vec.push_back(new PlanePlaneAngleConstraint(pl_fixed, pl_constrained, constraint["angle"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "angle_min_max")
            {
                constraints_vec.push_back(new PlanePlaneAngleMinMaxConstraint(pl_fixed, pl_constrained, constraint["angle_min"].asDouble(), constraint["angle_max"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else
            {
                errorMsg << "Constraint " << constraint_type << " not implemented for " << constraint_geoms;
                success = false;
                break;
            }
        }
        else if(constraint_geoms == "line_line")
        {
            bool flip = constraint["flip"].asBool();
            LineShapeDescriptor *l_constrained = new LineShapeDescriptor(readJsonVector(constraint["constrained"]["point"]),
                                                                 readJsonVector(constraint["constrained"]["normal"]));
            addGeometricEntitiy(rigid_object_constrained_id, l_constrained);
            LineShapeDescriptor *l_fixed = new LineShapeDescriptor(readJsonVector(constraint["fixed"]["point"]).transpose(),
                                                                 ((flip)? -1:1)*readJsonVector(constraint["fixed"]["normal"]));
            addGeometricEntitiy(rigid_object_fixed_id, l_fixed);
            if(constraint_type == "coincident")
            {
                constraints_vec.push_back(new LineLineCoincidentConstraint(l_fixed, l_constrained));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "distance")
            {
                constraints_vec.push_back(new LineLineParallelDistanceConstraint(l_fixed, l_constrained, constraint["distance"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "distance_min_max")
            {
                constraints_vec.push_back(new LineLineParallelDistanceMinMaxConstraint(l_fixed, l_constrained, constraint["distance_min"].asDouble(), constraint["distance_max"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "parallel")
            {
                constraints_vec.push_back(new LineLineParallelConstraint(l_fixed, l_constrained));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "angle")
            {
                constraints_vec.push_back(new LineLineAngleConstraint(l_fixed, l_constrained, constraint["angle"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else
            {
                errorMsg << "Constraint " << constraint_type << " not implemented for " << constraint_geoms;
                success = false;
                break;
            }
        }
        else if(constraint_geoms == "cylinder_cylinder")
        {
            bool flip = constraint["flip"].asBool();
            CylinderShapeDescriptor *cl_constrained = new CylinderShapeDescriptor(readJsonVector(constraint["constrained"]["point"]),
                                                                                readJsonVector(constraint["constrained"]["normal"]),
                                                                                constraint["constrained"]["radius"].asDouble(),
                                                                                constraint["constrained"]["height"].asDouble());
            addGeometricEntitiy(rigid_object_constrained_id, cl_constrained);
            CylinderShapeDescriptor *cl_fixed = new CylinderShapeDescriptor(readJsonVector(constraint["fixed"]["point"]),
                                                                                ((flip)? -1:1)*readJsonVector(constraint["fixed"]["normal"]),
                                                                                constraint["fixed"]["radius"].asDouble(),
                                                                                constraint["fixed"]["height"].asDouble());
            addGeometricEntitiy(rigid_object_fixed_id, cl_fixed);
            if(constraint_type == "coincident" || constraint_type == "concentric")
            {
                constraints_vec.push_back(new CylinderCylinderConcentricConstraint(cl_fixed, cl_constrained));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "distance")
            {
                constraints_vec.push_back(new CylinderCylinderParallelDistanceConstraint(cl_fixed, cl_constrained, constraint["distance"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "distance_min_max")
            {
                constraints_vec.push_back(new CylinderCylinderParallelDistanceMinMaxConstraint(cl_fixed, cl_constrained, constraint["distance_min"].asDouble(), constraint["distance_max"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "parallel")
            {
                constraints_vec.push_back(new CylinderCylinderParallelConstraint(cl_fixed, cl_constrained));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else if(constraint_type == "angle")
            {
                constraints_vec.push_back(new CylinderCylinderAngleConstraint(cl_fixed, cl_constrained, constraint["angle"].asDouble()));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
            else
            {
                errorMsg << "Constraint " << constraint_type << " not implemented for " << constraint_geoms;
                success = false;
                break;
            }
        }
        else if(constraint_geoms == "point_plane")
        {
            PlaneShapeDescriptor *pl_constrained = new PlaneShapeDescriptor(readJsonVector(constraint["constrained"]["point"]),
                                                                 readJsonVector(constraint["constrained"]["normal"]));
            addGeometricEntitiy(rigid_object_constrained_id, pl_constrained);
            PointShapeDescriptor *pt_fixed = new PointShapeDescriptor(readJsonVector(constraint["fixed"]["point"]).transpose());
            addGeometricEntitiy(rigid_object_fixed_id, pt_fixed);
            if(constraint_type == "coincident")
            {
                constraints_vec.push_back(new PointPlaneCoincidentConstraint(pt_fixed, pl_constrained));
                constraints_vec.back()->constrained_shape_id = rigid_object_constrained_id;
                constraints_vec.back()->fixed_shape_id = rigid_object_fixed_id;
            }
        }
    }
    fixedObjectID = root["fixedPoseId"].asInt();
    return rigid_objects[constrained_object_id];
}

RobotControl* ConstraintParser::parse_robot_controller_json(std::string constraints_json, std::stringstream &errorMsg)
{
    Json::Value root;
    Json::Reader reader;
    bool read_success = reader.parse(constraints_json, root, false);
    if(!read_success)
    {
        std::cout << "file couldn't be read" << std::endl;
        errorMsg << "file couldn't be read";
        return nullptr;
    }

    if(root.isMember("robot"))
    {
        Json::Value robot_description = root["robot"];
        return new RobotControl(robot_description["kinematic-file-url"].asString(), robot_description["robot-name"].asString(), robot_description["model-id"].asInt(), robot_description["coach-IP"].asString(), robot_description["dof"].asInt());
        std::cout << robot_description["kinematic-file-url"].asString() << std::endl;
        std::cout << robot_description["robot-name"].asString() << std::endl;
        std::cout << robot_description["model-id"].asInt() << std::endl;
        std::cout << robot_description["coach-IP"].asString() << std::endl;
        std::cout <<  robot_description["dof"].asInt() << std::endl;
    }
    else
    {
        std::cout << "no robot data" << std::endl;
        errorMsg << "no robot data";
        return nullptr;
    }
}

bool ConstraintParser::parse_environment_constraint_json(std::string constraints_json, CompositePrioritySkill* &skill_env, std::stringstream &errorMsg, std::string &solver_type, rl::mdl::Kinematic* &kinematic, ObjectModel* &shape_robot)
{
    Json::Value root;
    Json::Reader reader;
    bool read_success = reader.parse(constraints_json, root, false);
    if(!read_success)
    {
        std::cout << "file couldn't be read" << std::endl;
        errorMsg << "file couldn't be read";
        return false;
    }

    solver_type = root["solver"].asString();
    if(!root.isMember("env-constraints"))
    {
        errorMsg << "no environment constraints";
        return false;
    }

    Json::Value constraints = root["env-constraints"];
    int num_constraints = constraints.size();

    for(int constraint_id = 0;constraint_id < num_constraints;++constraint_id)
    {
        Json::Value constraint = constraints[constraint_id];

        std::string constraint_type = constraint["type"].asString();
        if(constraint_type == "collision")
        {
            std::string scene_fname = constraint["scene-fname"].asString();
            std::shared_ptr< rl::sg::Scene > collision_scene(new rl::sg::solid::Scene());
            collision_scene->load(scene_fname, true, true);
            std::vector<rl::sg::Body*> collision_bodies;
            Json::Value collision_body_ids = constraint["collision-body-ids"];
            for(int body_id = 0;body_id < collision_body_ids.size();++body_id)
            {
                Json::Value collision_body_id = collision_body_ids[body_id];
                collision_bodies.push_back(collision_scene->getModel(collision_body_id["model-id"].asInt())->getBody(collision_body_id["body-id"].asInt()));
            }
            CollisionAvoidanceSkill *skill_collision = new CollisionAvoidanceSkill(kinematic, collision_scene, shape_robot, collision_bodies);
            skill_env->addSkill(skill_collision);
        }
    }
    return true;
}

bool ConstraintParser::parse_collision_constraint_json(std::string constraints_json, std::stringstream &errorMsg, int &model_id, int &body_id)
{
    Json::Value root;
    Json::Reader reader;
    bool read_success = reader.parse(constraints_json, root, false);
    if(!read_success)
    {
        std::cout << "file couldn't be read" << std::endl;
        errorMsg << "file couldn't be read";
        return false;
    }

    if(!root.isMember("env-constraints"))
    {
        errorMsg << "no environment constraints";
        return false;
    }

    Json::Value constraints = root["env-constraints"];
    int num_constraints = constraints.size();

    bool found = false;
    for(int constraint_id = 0;constraint_id < num_constraints;++constraint_id)
    {
        Json::Value constraint = constraints[constraint_id];

        std::string constraint_type = constraint["type"].asString();
        if(constraint_type == "collision")
        {
            Json::Value collision_body_ids = constraint["collision-body-ids"];
            for(int coll_body_id = 0;coll_body_id < collision_body_ids.size();++coll_body_id)
            {
                Json::Value collision_body_id = collision_body_ids[coll_body_id];
                model_id = collision_body_id["model-id"].asInt();
                body_id = collision_body_id["body-id"].asInt();
            }
            found = true;
        }
    }
    return found;
}

bool ConstraintParser::parse_demo_params_json(std::string constraints_json, std::stringstream &errorMsg, bool &log, bool &exec, std::string &log_fname, std::vector<MoveSegment> &traj)
{
    Json::Value root;
    Json::Reader reader;
    bool read_success = reader.parse(constraints_json, root, false);
    if(!read_success)
    {
        std::cout << "file couldn't be read" << std::endl;
        errorMsg << "file couldn't be read";
        return false;
    }

    if(root.isMember("evaluation-params"))
    {
        Json::Value evaluation_params = root["evaluation-params"];
        log = evaluation_params["log"].asBool();
        log_fname = evaluation_params["log-fname"].asString();
        if(evaluation_params.isMember("exec"))
            exec = evaluation_params["exec"].asBool();
        else
            exec=false;
        if(evaluation_params.isMember("trajectory"))
        {
            Json::Value trajectory_json = evaluation_params["trajectory"];
            for(int segment_id = 0;segment_id < trajectory_json.size();++segment_id)
            {
                MoveSegment seg;
                if(trajectory_json[segment_id].isMember("object"))
                    seg.object = trajectory_json[segment_id]["object"].asString();
                else
                    seg.object = "robot";
                if(trajectory_json[segment_id].isMember("axis"))
                    seg.axis = trajectory_json[segment_id]["axis"].asString().c_str()[0];
                else
                    seg.axis = 'n';
                if(trajectory_json[segment_id].isMember("delta"))
                    seg.delta = trajectory_json[segment_id]["delta"].asDouble();
                else
                    seg.delta = 0.0;
                if(trajectory_json[segment_id].isMember("iter"))
                    seg.iter = trajectory_json[segment_id]["iter"].asInt();
                else
                    seg.iter = 0;
                traj.push_back(seg);
            }
        }
    }
    else
    {
        std::cout << "no evaluation data" << std::endl;
        errorMsg << "no evaluation data";
        return false;
    }
    return true;
}

bool ConstraintParser::parse_solver_params_json(std::string constraints_json, std::stringstream &errorMsg, SolverParams &params)
{
    Json::Value root;
    Json::Reader reader;
    bool read_success = reader.parse(constraints_json, root, false);
    if(!read_success)
    {
        std::cout << "file couldn't be read" << std::endl;
        errorMsg << "file couldn't be read";
        return false;
    }

    if(root.isMember("solver-params"))
    {
        Json::Value solver_params_json = root["solver-params"];
        if(solver_params_json.isMember("f-tol"))
            params.f_tol_ = solver_params_json["f-tol"].asDouble();
        if(solver_params_json.isMember("x-tol"))
            params.x_tol_ = solver_params_json["x-tol"].asDouble();
        if(solver_params_json.isMember("f-tol"))
            params.constraint_tol_ = solver_params_json["constraint-tol"].asDouble();
        if(solver_params_json.isMember("stopval"))
            params.stopval_ = solver_params_json["stopval"].asDouble();
        if(solver_params_json.isMember("max-time"))
            params.max_time_ = solver_params_json["max-time"].asDouble();
    }
    else
    {
        std::cout << "no evaluation data" << std::endl;
        errorMsg << "no evaluation data";
        return false;
    }
    return true;
}
