/**
 * Python wrappers for raisim.object.ArticulatedSystem using pybind11.
 *
 * Copyright (c) 2019, jhwangbo (C++), Brian Delhaisse <briandelhaisse@gmail.com> (Python wrappers)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>     // automatic conversion between std::vector, std::list, etc to Python list/tuples/dict
#include <pybind11/eigen.h>   // automatic conversion between Eigen data types to Numpy data types

#include "raisim/math.hpp"   // contains the definitions of Vec, Mat, and others, as well as transformations.
#include "raisim/object/ArticulatedSystem/loaders.hpp"
#include "raisim/object/ArticulatedSystem/JointAndBodies.hpp"
#include "raisim/object/ArticulatedSystem/ArticulatedSystem.hpp"

#include "converter.hpp"  // contains code that allows to convert between the Vec, Mat to numpy arrays.

#include "ode/collision.h"
#include "ode/ode.h"
#include "ode/extras/collision_kernel.h"
// Important note: for the above include ("ode/src/collision_kernel.h"), you have to add a `extras` folder in the
// `$LOCAL_BUILD/include/ode/` which should contain the following header files:
// array.h, collision_kernel.h, common.h, error.h, objects.h, odeou.h, odetls.h, threading_base.h, and typedefs.h.
// These header files can be found in the `ode/src` folder (like here: https://github.com/thomasmarsh/ODE/tree/master/ode/src)
//
// Why do we need to do that? The reason is that for `raisim::Mesh`, the authors of RaiSim use the `dSpaceID` variable
// type which has been forward declared in `ode/common.h` (but not defined there) as such:
//
// struct dxSpace;
// typedef struct dxSpace *dSpaceID;
//
// Thus for `dSpaceID` we need the definition of `dxSpace`, and this one is defined in `ode/src/collision_kernel.h` (in
// the `src` folder and not in the `include` folder!!). Pybind11 is looking for that definition, if you don't include
// it, pybind11 will complain and raise errors.


namespace py = pybind11;
using namespace raisim;


void init_articulated_system(py::module &m) { // py::module &main_module) {


    /****************/
    /* LoadFromMJCF */
    /****************/
    py::class_<raisim::mjcf::LoadFromMJCF>(m, "LoadFromMJCF", "Load from MJCF file.")
        .def(py::init<ArticulatedSystem &, std::string, std::vector<std::string>>(), "Initialize the MJCF loader.");


    /*****************/
    /* LoadFromURDF2 */
    /*****************/
    py::class_<raisim::urdf::LoadFromURDF2>(m, "LoadFromURDF2", "Load from URDF file.")
        .def(py::init<ArticulatedSystem &, std::string, std::vector<std::string>>(), "Initialize the URDF loader.");


    /***************/
    /* ControlMode */
    /***************/
    py::enum_<raisim::ControlMode::Type>(m, "ControlMode", py::arithmetic())
        .value("FORCE_AND_TORQUE", raisim::ControlMode::Type::FORCE_AND_TORQUE)
        .value("PD_PLUS_FEEDFORWARD_TORQUE", raisim::ControlMode::Type::PD_PLUS_FEEDFORWARD_TORQUE)
        .value("VELOCITY_PLUS_FEEDFORWARD_TORQUE", raisim::ControlMode::Type::VELOCITY_PLUS_FEEDFORWARD_TORQUE);


    /***********************/
    /* CollisionDefinition */
    /***********************/
    py::class_<raisim::CollisionDefinition>(m, "CollisionDefinition", "Raisim CollisionDefinition struct.")

        .def(py::init<>(), "Initialize the Collision Definition class.")

        .def(py::init([](py::array_t<double> rot_offset, py::array_t<double> pos_offset, size_t local_idx,
                dGeomID collision_object, std::string name) {
            // convert from np to Mat and Vec
            raisim::Mat<3,3> rot = convert_np_to_mat<3,3>(rot_offset);
            raisim::Vec<3> pos = convert_np_to_vec<3>(pos_offset);

            // construct
            return new raisim::CollisionDefinition(rot, pos, local_idx, collision_object, name);
        }),  R"mydelimiter(
        Instantiate the Collision Definition class.

        Args:
            rot_offset (np.array[float[3,3]]): rotation offset matrix.
            pos_offset (np.array[float[3]]): position offset.
            local_idx (int): local index.
            collision_object (dGeomID): collision object.
            name (str): name of the collision definition.
        )mydelimiter",
        py::arg("rot_offset"), py::arg("pos_offset"), py::arg("local_idx"), py::arg("collision_object"), py::arg("name"))

        .def_property("rot_offset",
            [](raisim::CollisionDefinition &self) {  // getter
                return convert_mat_to_np(self.rotOffset);
            }, [](raisim::CollisionDefinition &self, py::array_t<double> array) {  // setter
                Mat<3,3> rot = convert_np_to_mat<3,3>(array);
                self.rotOffset = rot;
            })
        .def_property("pos_offset",
            [](raisim::CollisionDefinition &self) {  // getter
                return convert_vec_to_np(self.posOffset);
            }, [](raisim::CollisionDefinition &self, py::array_t<double> array) {  // setter
                Vec<3> pos = convert_np_to_vec<3>(array);
                self.posOffset = pos;
            })
        .def_readwrite("local_idx", &raisim::CollisionDefinition::localIdx)
        .def_readwrite("collision_object", &raisim::CollisionDefinition::colObj)
        .def_readwrite("name", &raisim::CollisionDefinition::name);


    /*********/
    /* Shape */
    /*********/
    py::enum_<raisim::Shape::Type>(m, "ShapeType", py::arithmetic())
        .value("Box", raisim::Shape::Type::Box)
        .value("Cylinder", raisim::Shape::Type::Cylinder)
        .value("Sphere", raisim::Shape::Type::Sphere)
        .value("Mesh", raisim::Shape::Type::Mesh)
        .value("Capsule", raisim::Shape::Type::Capsule)
        .value("Cone", raisim::Shape::Type::Cone);


    /*************/
    /* VisObject */
    /*************/
    py::class_<raisim::VisObject>(m, "VisObject", "Raisim Visualization Object struct.")
        .def_property("rot",
            [](raisim::VisObject &self) {  // getter
                return convert_mat_to_np(self.rot);
            }, [](raisim::VisObject &self, py::array_t<double> array) {  // setter
                Mat<3,3> rot = convert_np_to_mat<3,3>(array);
                self.rot = rot;
            })
        .def_property("offset",
            [](raisim::VisObject &self) {  // getter
                return convert_vec_to_np(self.offset);
            }, [](raisim::VisObject &self, py::array_t<double> array) {  // setter
                Vec<3> pos = convert_np_to_vec<3>(array);
                self.offset = pos;
            })
        .def_property("scale",
            [](raisim::VisObject &self) {  // getter
                return convert_vec_to_np(self.scale);
            }, [](raisim::VisObject &self, py::array_t<double> array) {  // setter
                Vec<3> pos = convert_np_to_vec<3>(array);
                self.scale = pos;
            })
        .def_property("color",
            [](raisim::VisObject &self) {  // getter
                return convert_vec_to_np(self.color);
            }, [](raisim::VisObject &self, py::array_t<double> array) {  // setter
                Vec<4> pos = convert_np_to_vec<4>(array);
                self.color = pos;
            })
        .def_readwrite("local_idx", &raisim::VisObject::localIdx)
        .def_readwrite("shape", &raisim::VisObject::shape)
        .def_readwrite("filename", &raisim::VisObject::fileName)
        .def_readwrite("material", &raisim::VisObject::material)
        .def_readwrite("vis_shape_param", &raisim::VisObject::visShapeParam)
        .def_readwrite("name", &raisim::VisObject::name);


    /*********/
    /* Joint */
    /*********/
    py::class_<raisim::Joint> joint(m, "Joint", "Raisim Joint struct.");

    py::enum_<raisim::Joint::Type>(joint, "Type")
        .value("FIXED", raisim::Joint::Type::FIXED)
        .value("REVOLUTE", raisim::Joint::Type::REVOLUTE)
        .value("PRISMATIC", raisim::Joint::Type::PRISMATIC)
        .value("SPHERICAL", raisim::Joint::Type::SPHERICAL)
        .value("FLOATING", raisim::Joint::Type::FLOATING);

    joint.def(py::init<>(), "Instantiate the Joint class.")

        .def("joint_axis", &raisim::Joint::jointAxis, R"mydelimiter(
        Set the given joint axis.

        Args:
            axis (list of 3 float): joint axis.
        )mydelimiter",
        py::arg("axis"))

        .def("joint_position", py::overload_cast<std::initializer_list<double>>(&raisim::Joint::jointPosition), R"mydelimiter(
        Set the joint position.

        Args:
            position (list of 3 float): joint position.
        )mydelimiter",
        py::arg("position"))

        .def("joint_position", [](raisim::Joint &self, py::array_t<double> position) {
            Vec<3> pos = convert_np_to_vec<3>(position);
            self.jointPosition(pos);
        }, R"mydelimiter(
        Set the joint position.

        Args:
            position (list of 3 float): joint position.
        )mydelimiter",
        py::arg("position"))

        .def("get_gc_dim", &raisim::Joint::getGcDim, R"mydelimiter(
        Get the dimension for the generalized coordinates (gc) associated with the joint type.

        Returns:
            int: dimension
        )mydelimiter")

        .def("get_gv_dim", &raisim::Joint::getGvDim, R"mydelimiter(
        Get the dimension for the generalized velocities (gv) associated with the joint type.

        Returns:
            int: dimension
        )mydelimiter")

        .def_property("rot",
            [](raisim::Joint &self) {  // getter
                return convert_mat_to_np(self.rot);
            }, [](raisim::Joint &self, py::array_t<double> array) {  // setter
                Mat<3,3> rot = convert_np_to_mat<3,3>(array);
                self.rot = rot;
            })

        .def_property("limit",
            [](raisim::Joint &self) {  // getter
                return convert_vec_to_np(self.limit);
            }, [](raisim::Joint &self, py::array_t<double> array) {  // setter
                auto limit = convert_np_to_vec<2>(array);
                self.limit = limit;
            })

        .def_property("axis",
            [](raisim::Joint &self) {  // getter
                return convert_vec_to_np(self.axis_);
            }, [](raisim::Joint &self, py::array_t<double> array) {  // setter
                auto axis = convert_np_to_vec<3>(array);
                self.axis_ = axis;
            })

        .def_property("pos",
            [](raisim::Joint &self) {  // getter
                return convert_vec_to_np(self.pos_P_);
            }, [](raisim::Joint &self, py::array_t<double> array) {  // setter
                auto pos = convert_np_to_vec<3>(array);
                self.pos_P_ = pos;
            })

        .def_readwrite("type", &raisim::Joint::type);


    /*******************/
    /* CoordinateFrame */
    /*******************/
    py::class_<raisim::CoordinateFrame>(m, "CoordinateFrame", "Raisim Coordinate Frame class.")
        .def_property("position",
            [](raisim::CoordinateFrame &self) {  // getter
                return convert_vec_to_np(self.position);
            }, [](raisim::CoordinateFrame &self, py::array_t<double> array) {  // setter
                auto pos = convert_np_to_vec<3>(array);
                self.position = pos;
            })
        .def_property("orientation",
            [](raisim::CoordinateFrame &self) {  // getter
                return convert_mat_to_np(self.orientation);
            }, [](raisim::CoordinateFrame &self, py::array_t<double> array) {  // setter
                Mat<3,3> rot = convert_np_to_mat<3,3>(array);
                self.orientation = rot;
            })
        .def_readwrite("parent_id", &raisim::CoordinateFrame::parentId)
        .def_readwrite("parent_name", &raisim::CoordinateFrame::parentName)
        .def_readwrite("name", &raisim::CoordinateFrame::name)
        .def_readwrite("is_child", &raisim::CoordinateFrame::isChild);  // child is the first body after movable joint. All fixed bodies attached to a child is not a child


    /********/
    /* Body */
    /********/
    py::class_<raisim::Body>(m, "Body", "Raisim Body class.")
        .def(py::init<>(), "Instantiate the body class.")

        .def("set_mass", py::overload_cast<double>(&raisim::Body::mass), R"mydelimiter(
        Set the body mass.

        Args:
            mass (float): body mass.
        )mydelimiter",
        py::arg("mass"))
        .def("get_mass", py::overload_cast<>(&raisim::Body::mass), R"mydelimiter(
        Return the body mass.

        Returns:
            float: body mass.
        )mydelimiter")


        .def("set_inertia", py::overload_cast<std::initializer_list<double>>(&raisim::Body::inertia), R"mydelimiter(
        Set the body full inertia matrix.

        Args:
            inertia (list[float]): full inertia matrix [ixx, ixy, ixz, iyy, iyz, izz]
        )mydelimiter",
        py::arg("inertia"))


        .def("set_inertia", [](raisim::Body &self, py::array_t<double> inertia) {
            if (inertia.size() == 3) {
                Vec<3> I = convert_np_to_vec<3>(inertia);
                self.inertia(I);
            } else if (inertia.size() == 9) {
                // check dimensions and shape
                if (inertia.ndim() != 2) {
                    std::ostringstream s;
                    s << "error: expecting the given inertia to have a dimension of 2, but got instead a dimension of "
                        << inertia.ndim() << ".";
                    throw std::domain_error(s.str());
                }
                if ((inertia.shape(0) != 3) || (inertia.shape(1) != 3)) {
                    std::ostringstream s;
                    s << "error: expecting the given inertia to have the shape (3,3), but got instead the shape ("
                        << inertia.shape(0) << ", " << inertia.shape(1) << ").";
                    throw std::domain_error(s.str());
                }

                Vec<6> I;
                I[0] = *inertia.data(0, 0);  // ixx
                I[1] = *inertia.data(0, 1);  // ixy
                I[2] = *inertia.data(0, 2);  // ixz
                I[3] = *inertia.data(1, 1);  // iyy
                I[4] = *inertia.data(1, 2);  // iyz
                I[5] = *inertia.data(2, 2);  // izz
                self.inertia(I);
            } else {
                Vec<6> I = convert_np_to_vec<6>(inertia);
                self.inertia(I);
            }
        }, R"mydelimiter(
        Set the body inertia matrix.

        Args:
            inertia (np.array[float[3]], np.array[float[6]], np.array[float[3,3]]): inertia matrix given as
                [ixx, iyy, izz], [ixx, ixy, ixz, iyy, iyz, izz], or [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]].
        )mydelimiter",
        py::arg("inertia"))


        .def("get_inertia", [](raisim::Body &self) {
            Mat<3,3> inertia = self.inertia();
            return convert_mat_to_np(inertia);
        } , R"mydelimiter(
        Return the body full inertia matrix.

        Returns:
            np.array[float[3,3]]: full inertia matrix
        )mydelimiter")


        .def("set_com", py::overload_cast<std::initializer_list<double>>(&raisim::Body::com), R"mydelimiter(
        Set the body center of mass position.

        Args:
            com (list[float]): com position.
        )mydelimiter",
        py::arg("com"))

        .def("set_com", [](raisim::Body &self, py::array_t<double> com) {
            Vec<3> pos = convert_np_to_vec<3>(com);
            self.com(pos);
        }, R"mydelimiter(
        Set the body center of mass position.

        Args:
            com (np.array[float[3]]): com position.
        )mydelimiter",
        py::arg("com"))

        .def("get_com", [](raisim::Body &self) {
            Vec<3> com = self.com();
            return convert_vec_to_np(com);
        } , R"mydelimiter(
        Return the body center of mass position.

        Returns:
            np.array[float[3]]: com position.
        )mydelimiter")


        .def("set_rpy", [](raisim::Body &self, py::array_t<double> rpy) {
            self.RPY(convert_np_to_vec<3>(rpy));
        }, R"mydelimiter(
        Set the body's orientation as roll-pitch-yaw angles.

        Args:
            rpy (np.array[float[3]]): roll-pitch-yaw angles.
        )mydelimiter",
        py::arg("rpy"))

        .def("get_rpy", [](raisim::Body &self) {
            Vec<3> rpy = self.RPY();
            return convert_vec_to_np(rpy);
        } , R"mydelimiter(
        Return the body's orientation as roll-pitch-yaw angles.

        Returns:
            np.array[float[3]]: roll-pitch-yaw angles.
        )mydelimiter")


        .def("get_rotation_matrix", [](raisim::Body &self) {
            auto rot = self.rotationMatrix();
            return convert_mat_to_np(rot);
        } , R"mydelimiter(
        Return the body's orientation as a rotation matrix.

        Returns:
            np.array[float[3, 3]]: rotation matrix.
        )mydelimiter")


        .def("set_collision_orientation", [](raisim::Body &self, py::array_t<double> array) {
            if (array.size() == 3) {
                Vec<3> rpy = convert_np_to_vec<3>(array);
                self.collisionOrientation(rpy);
            } else if (array.size() == 9) {
                Mat<3,3> rot = convert_np_to_mat<3,3>(array);
                self.collisionOrientation(rot);
            } else {
                std::ostringstream s;
                s << "error: expecting the given array to have a size of 3 or 9, but got instead a size of "
                    << array.size() << ".";
                throw std::domain_error(s.str());
            }
        }, R"mydelimiter(
        Set the body's collision orientation.

        Args:
            orientation (np.array[float[3]], np.array[float[3,3]]): orientation (expressed as roll-pitch-yaw angles, or
                rotation matrix).
        )mydelimiter",
        py::arg("orientation"))


        .def("set_visualization_orientation", [](raisim::Body &self, py::array_t<double> array) {
            Vec<3> rpy = convert_np_to_vec<3>(array);
            self.visOrientation(rpy);
        }, R"mydelimiter(
        Set the body's visualization orientation.

        Args:
            rpy (np.array[float[3]]): orientation (expressed as roll-pitch-yaw angles).
        )mydelimiter",
        py::arg("rpy"))

        .def("clear_collision_and_visualization", &raisim::Body::clearColAndVis, "Clear the collision and visualization.")

        .def_readwrite("collision_shapes", &raisim::Body::colshape)
        .def_readwrite("visualization_shapes", &raisim::Body::visshape)
        .def_readwrite("visualization_shape_parameters", &raisim::Body::visShapeParam)
        .def_readwrite("mesh_filenames", &raisim::Body::meshFileNames)
        .def_readwrite("collision_shape_parameters", &raisim::Body::colShapeParam)
        .def_readwrite("material_names", &raisim::Body::materialName)
        .def_readwrite("collision_visualized_materials", &raisim::Body::collisionVisualizedMaterial)
        .def_readwrite("visualization_names", &raisim::Body::visName)
        .def_readwrite("collision_names", &raisim::Body::colName)
        .def_readwrite("collision_mesh_filenames", &raisim::Body::colMeshFileName)
        .def_readwrite("mass", &raisim::Body::mass_)
        .def_readwrite("is_colliding", &raisim::Body::isColliding)

        .def_property("combined_collision_position",
            [](raisim::Body &self) {  // getter
                return convert_vec_to_np(self.combinedColPos);
            }, [](raisim::Body &self, py::array_t<double> array) {  // setter
                auto pos = convert_np_to_vec<3>(array);
                self.combinedColPos = pos;
            })

        .def_property("combined_collision_rotation_matrix",
            [](raisim::Body &self) {  // getter
                return convert_mat_to_np(self.combinedColRotMat);
            }, [](raisim::Body &self, py::array_t<double> array) {  // setter
                Mat<3,3> rot = convert_np_to_mat<3,3>(array);
                self.combinedColRotMat = rot;
            })

        .def_property("com",
            [](raisim::Body &self) {  // getter
                return convert_vec_to_np(self.com_);
            }, [](raisim::Body &self, py::array_t<double> array) {  // setter
                auto pos = convert_np_to_vec<3>(array);
                self.com_ = pos;
            })

        .def_property("rpy",
            [](raisim::Body &self) {  // getter
                return convert_vec_to_np(self.rollPitchYaw);
            }, [](raisim::Body &self, py::array_t<double> array) {  // setter
                auto rpy = convert_np_to_vec<3>(array);
                self.rollPitchYaw = rpy;
            })

        .def_property("rotation_matrix",
            [](raisim::Body &self) {  // getter
                return convert_mat_to_np(self.rotMat_);
            }, [](raisim::Body &self, py::array_t<double> array) {  // setter
                Mat<3,3> rot = convert_np_to_mat<3,3>(array);
                self.rotMat_ = rot;
            })

        .def_property("collision_rotation_matrix",
            [](raisim::Body &self) {  // getter
                return convert_mat_to_np(self.collisionRotMat_);
            }, [](raisim::Body &self, py::array_t<double> array) {  // setter
                Mat<3,3> rot = convert_np_to_mat<3,3>(array);
                self.collisionRotMat_ = rot;
            })

        .def_property("inertia",
            [](raisim::Body &self) {  // getter
                return convert_mat_to_np(self.inertia_);
            }, [](raisim::Body &self, py::array_t<double> array) {  // setter
                Mat<3,3> I = convert_np_to_mat<3,3>(array);
                self.inertia_ = I;
            })

        .def_property("collision_object_origin",  // (avoid to use this one as we have to copy everything)
            [](raisim::Body &self) { // getter
                py::list list;
                for (auto elem : self.colObjOrigin)
                    list.append(convert_vec_to_np(elem));
                return list;
            }, [](raisim::Body &self, py::list list) { // setter
                std::vector<raisim::Vec<3> > vector;
                for (auto elem : list) {
                    py::array_t<double> e = elem.cast<py::array_t<double>>();
                    vector.push_back(convert_np_to_vec<3>(e));
                }
                self.colObjOrigin = vector;
            })

        .def_property("visualization_object_origin",  // (avoid to use this one as we have to copy everything)
            [](raisim::Body &self) { // getter
                py::list list;
                for (auto elem : self.visObjOrigin)
                    list.append(convert_vec_to_np(elem));
                return list;
            }, [](raisim::Body &self, py::list list) { // setter
                std::vector<raisim::Vec<3> > vector;
                for (auto elem : list) {
                    py::array_t<double> e = elem.cast<py::array_t<double>>();
                    vector.push_back(convert_np_to_vec<3>(e));
                }
                self.visObjOrigin = vector;
            })

        .def_property("visualization_scales",  // (avoid to use this one as we have to copy everything)
            [](raisim::Body &self) { // getter
                py::list list;
                for (auto elem : self.visScale)
                    list.append(convert_vec_to_np(elem));
                return list;
            }, [](raisim::Body &self, py::list list) { // setter
                std::vector<raisim::Vec<3> > vector;
                for (auto elem : list) {
                    py::array_t<double> e = elem.cast<py::array_t<double>>();
                    vector.push_back(convert_np_to_vec<3>(e));
                }
                self.visScale = vector;
            })

        .def_property("visualization_colors",  // (avoid to use this one as we have to copy everything)
            [](raisim::Body &self) { // getter
                py::list list;
                for (auto elem : self.visColor)
                    list.append(convert_vec_to_np(elem));
                return list;
            }, [](raisim::Body &self, py::list list) { // setter
                std::vector<raisim::Vec<4> > vector;
                for (auto elem : list) {
                    py::array_t<double> e = elem.cast<py::array_t<double>>();
                    vector.push_back(convert_np_to_vec<4>(e));
                }
                self.visColor = vector;
            })

        .def_property("collision_rotation_matrices",
            [](raisim::Body &self) { // getter
                py::list list;
                for (auto elem : self.collisionRotMat)
                    list.append(convert_mat_to_np(elem));
                return list;
            }, [](raisim::Body &self, py::list list) { // setter
                std::vector<raisim::Mat<3,3>> vector;
                for (auto elem : list) {
                    py::array_t<double> e = elem.cast<py::array_t<double>>();
                    vector.push_back(convert_np_to_mat<3, 3>(e));
                }
                self.collisionRotMat = vector;
            })

        .def_property("visualization_rotation_matrices",
            [](raisim::Body &self) { // getter
                py::list list;
                for (auto elem : self.visRotMat)
                    list.append(convert_mat_to_np(elem));
                return list;
            }, [](raisim::Body &self, py::list list) { // setter
                std::vector<raisim::Mat<3,3>> vector;
                for (auto elem : list) {
                    py::array_t<double> e = elem.cast<py::array_t<double>>();
                    vector.push_back(convert_np_to_mat<3, 3>(e));
                }
                self.visRotMat = vector;
            });


    /*********/
    /* Child */
    /*********/
    py::class_<raisim::Child>(m, "Child", "Raisim Child class.")
        .def_readwrite("body_idx", &raisim::Child::bodyIdx)
        .def_readwrite("parent_idx", &raisim::Child::parentIdx)
        .def_readwrite("body", &raisim::Child::body)
        .def_readwrite("joint", &raisim::Child::joint)
        .def_readwrite("registered", &raisim::Child::registered)
        .def_readwrite("name", &raisim::Child::name)
        .def_readwrite("parent_name", &raisim::Child::parentName)
        .def_readwrite("parent_joint_name", &raisim::Child::parentJointName)
        .def_readwrite("children", &raisim::Child::child)
        .def_readwrite("fixed_bodies", &raisim::Child::fixedBodies)

        .def("number_of_bodies_from_here", &raisim::Child::numberOfBodiesFromHere, R"mydelimiter(
        Get the number of bodies from this child body.

        Returns:
            int: number of bodies.
        )mydelimiter")


        .def("number_of_dof_from_here", &raisim::Child::numberOfDOFFromHere, R"mydelimiter(
        Get the number of degrees of freedom from this child body.

        Returns:
            int: number of bodies.
        )mydelimiter")


        .def("number_of_gc_from_here", &raisim::Child::numberOfGCFromHere, R"mydelimiter(
        Get the number of generalized coordinates from this child body.

        Returns:
            int: number of bodies.
        )mydelimiter")


        .def("joint_idx", &raisim::Child::jointIdx, R"mydelimiter(
        Get the joint index corresponding to the given joint name from a list of joint names.

        Args:
            name (str): the joint name we want in the given list of joint names.
            names (list[str]): list of joint names.

        Returns:
            int: joint index. (-1 if it couldn't find it)
        )mydelimiter",
        py::arg("name"), py::arg("names"))


        .def("init_collision_bodies", &raisim::Child::initCollisionBodies, R"mydelimiter(
        Initialize the visual objects.

        Args:
            collisions (list[CollisionDefinition]): list of collision definitions.
            visual_objects (list[VisObject]): list of visualization objects.
            space (dSpaceID): collision space.
            meshes (list[Mesh]): list of meshes.
            resource_directory (str): resource directory.
        )mydelimiter",
        py::arg("collisions"), py::arg("visual_objects"), py::arg("space"), py::arg("meshes"), py::arg("resource_directory"))


        .def("init_visuals", &raisim::Child::initVisuals, R"mydelimiter(
        Initialize the visual objects.

        Args:
            objects (list[VisObject]): list of visualization objects.
        )mydelimiter",
        py::arg("objects"))


        .def("consume_fixed_bodies", &raisim::Child::consumeFixedBodies, R"mydelimiter(
        Consume the fixed bodies.

        Args:
            frames (list[CoordinateFrame]): list of coordinate frames that interest us.
            joint_names (list[str]): list of joint names.
        )mydelimiter",
        py::arg("frames"), py::arg("joint_names"));



    /***************************/
    /* ArticulatedSystemOption */
    /***************************/
    py::class_<raisim::ArticulatedSystemOption>(m, "ArticulatedSystemOption", "Raisim Articulated System Option.")
        .def_readwrite("do_not_collide_with_parent", &raisim::ArticulatedSystemOption::doNotCollideWithParent);


    /*********************/
    /* ArticulatedSystem */
    /*********************/

    // From the `ArticulatedSystem.h` file:
    /* list of vocabs
     1. body: body here refers to only rotating bodies. Fixed bodies are optimized out.
              Position of a body refers to the position of the joint connecting the body and its parent.
     2. Coordinate frame: coordinate frames are defined on every joint (even at the fixed joints). If you want
                          to define a custom frame, define a fixed zero-mass object and a joint in the URDF */

    py::class_<raisim::ArticulatedSystem, raisim::Object> system(m, "ArticulatedSystem", "Raisim Articulated System.");

    py::enum_<raisim::ArticulatedSystem::Frame>(system, "Frame")
        .value("WORLD_FRAME", raisim::ArticulatedSystem::Frame::WORLD_FRAME)
        .value("PARENT_FRAME", raisim::ArticulatedSystem::Frame::PARENT_FRAME)
        .value("BODY_FRAME", raisim::ArticulatedSystem::Frame::BODY_FRAME);

    system.def(py::init<>(), "Initialize the Articulated System.")

        .def(py::init<const std::string &, const std::string &, std::vector<std::string>, raisim::ArticulatedSystemOption>(),
        "Initialize the Articulated System.\n\n"
        "Do not call this method yourself. use World class to create an Articulated system.\n\n"
        "Args:\n"
        "    filename (str): path to the robot description file (URDF, etc).\n"
        "    resource_directory (str): path the resource directory. If empty, it will use the robot description folder.\n"
        "    joint_order (list[str]): specify the joint order, if we want it to be different from the URDF file.\n"
        "    options (ArticulatedSystemOption): options.",
        py::arg("filename"), py::arg("resource_directory"), py::arg("joint_order"), py::arg("options"))


        .def("get_generalized_coordinates", [](raisim::ArticulatedSystem &self) {
            return convert_vecdyn_to_np(self.getGeneralizedCoordinate());
        }, R"mydelimiter(
        Get the generalized coordinates of the system.

        The dimension of the returned vector is equal to the number of degrees of freedom that each joint provides.
        A floating joint provides 6 DoFs (3 linear + 3 revolute), a prismatic/revolute joint 1 DoF, a fixed joint 0
        DoF, etc.

        Returns:
            np.array[float[n]]: generalized coordinates.
        )mydelimiter")


        .def("get_base_quaternion", [](raisim::ArticulatedSystem &self) {
            Vec<4> quaternion;
            self.getBaseOrientation(quaternion);
            return convert_vec_to_np(quaternion);
        }, R"mydelimiter(
        Get the base orientation (expressed as a quaternion [w,x,y,z]).

        Returns:
            np.array[float[4]]: base orientation (expressed as a quaternion [w,x,y,z])
        )mydelimiter")


        .def("get_base_rotation_matrix", [](raisim::ArticulatedSystem &self) {
            Mat<3,3> rot;
            self.getBaseOrientation(rot);
            return convert_mat_to_np(rot);
        }, R"mydelimiter(
        Get the base orientation (expressed as a rotation matrix).

        Returns:
            np.array[float[3,3]]: rotation matrix
        )mydelimiter")


        .def("get_generalized_velocities", [](raisim::ArticulatedSystem &self) {
            return convert_vecdyn_to_np(self.getGeneralizedVelocity());
        }, R"mydelimiter(
        Get the generalized velocities of the system.

        The dimension of the returned vector is equal to the number of degrees of freedom that each joint provides.
        A floating joint provides 6 DoFs (3 linear + 3 revolute), a prismatic/revolute joint 1 DoF, a fixed joint 0
        DoF, etc.

        Returns:
            np.array[float[n]]: generalized velocities.
        )mydelimiter")


        .def("update_kinematics", &raisim::ArticulatedSystem::updateKinematics,  R"mydelimiter(
        Update the kinematics.

        It is unnecessary to call this function if you are simulating your system. `integrate1` calls this function.
        Call this function if you want to get kinematic properties but you don't want to integrate.
        )mydelimiter")


        .def("set_generalized_coordinates", py::overload_cast<std::initializer_list<double>>(&raisim::ArticulatedSystem::setGeneralizedCoordinate), R"mydelimiter(
        Set the generalized coordinates.

        Args:
            coordinates (list[float]): generalized coordinates to set.
        )mydelimiter",
        py::arg("coordinates"))
        .def("set_generalized_coordinates", py::overload_cast<const Eigen::VectorXd &>(&raisim::ArticulatedSystem::setGeneralizedCoordinate), R"mydelimiter(
        Set the generalized coordinates.

        Args:
            coordinates (np.array[float[n]]): generalized coordinates to set.
        )mydelimiter",
        py::arg("coordinates"))


        .def("set_generalized_velocities", py::overload_cast<std::initializer_list<double>>(&raisim::ArticulatedSystem::setGeneralizedVelocity), R"mydelimiter(
        Set the generalized velocities.

        Args:
            velocities (list[float]): generalized velocities to set.
        )mydelimiter",
        py::arg("velocities"))
        .def("set_generalized_velocities", py::overload_cast<const Eigen::VectorXd &>(&raisim::ArticulatedSystem::setGeneralizedVelocity), R"mydelimiter(
        Set the generalized velocities.

        Args:
            velocities (np.array[float[n]]): generalized velocities to set.
        )mydelimiter",
        py::arg("velocities"))


        .def("set_generalized_forces", py::overload_cast<std::initializer_list<double>>(&raisim::ArticulatedSystem::setGeneralizedForce), R"mydelimiter(
        Set the generalized forces.

        These are the feedforward generalized forces. In the PD control mode, this differs from the actual
        generalizedForces. The dimension should be the same as the number of DoFs.

        Args:
            forces (list[float]): generalized forces to set.
        )mydelimiter",
        py::arg("forces"))
        .def("set_generalized_forces", py::overload_cast<const Eigen::VectorXd &>(&raisim::ArticulatedSystem::setGeneralizedForce), R"mydelimiter(
        Set the generalized forces.

        These are the feedforward generalized forces. In the PD control mode, this differs from the actual
        generalizedForces. The dimension should be the same as the number of DoFs.

        Args:
            forces (np.array[float[n]]): generalized forces to set.
        )mydelimiter",
        py::arg("forces"))
//        .def("set_generalized_forces", [](raisim::ArticulatedSystem &self, py::array_t<double> forces) {
//            raisim::VecDyn tau = convert_np_to_vecdyn(forces);
//            self.setGeneralizedForce(tau);
//        }, R"mydelimiter(
//        Set the generalized forces.
//
//        Args:
//            forces (list[float]): generalized forces to set.
//        )mydelimiter",
//        py::arg("forces"))


        .def("get_states", [](raisim::ArticulatedSystem &self) {
            Eigen::VectorXd gc;
            Eigen::VectorXd gv;

            self.getState(gc, gv);

            return py::make_tuple(gc, gv);
        }, R"mydelimiter(
        Get the joint states.

        Returns:
            np.array[float[n]]: generalized coordinates.
            np.array[float[n]]: generalized velocities.
        )mydelimiter")


        .def("set_states", &raisim::ArticulatedSystem::setState, R"mydelimiter(
        Set the joint states.

        Args:
            coordinates (np.array[float[n]]): generalized coordinates to set.
            velocities (np.array[float[n]]): generalized velocities to set.
        )mydelimiter",
        py::arg("coordinates"), py::arg("velocities"))


        /* get dynamics properties. Make sure that after integration you call "integrate1()" of the world object
        before using this method. Generalized force is the actual force */
        .def("get_generalized_forces", [](raisim::ArticulatedSystem &self) {
            VecDyn vec = self.getGeneralizedForce();
            return convert_vecdyn_to_np(vec);
        }, R"mydelimiter(
        Get the generalized forces.

        Returns:
            np.array[float[n]]: generalized forces.
        )mydelimiter")


        .def("get_feedforward_generalized_forces", [](raisim::ArticulatedSystem &self) {
            VecDyn vec = self.getFeedForwardGeneralizedForce();
            return convert_vecdyn_to_np(vec);
        }, R"mydelimiter(
        Get the feedforward generalized forces.

        Returns:
            np.array[float[n]]: feedforward generalized forces.
        )mydelimiter")


        .def("get_mass_matrix", [](raisim::ArticulatedSystem &self) {
            MatDyn mat = self.getMassMatrix();
            return convert_matdyn_to_np(mat);
        }, R"mydelimiter(
        Get the mass (inertia) matrix which is present in the dynamic equation of motion:

        .. math:: H(q) \ddot{q} + N(q, \dot{q}) = \tau

        where :math:`H(q)` is the mass inertia matrix, :math:`N(q, \dot{q})` are the non-linear terms, and
        :math:`\tau` are the forces/torques.

        Returns:
            np.array[float[n,n]]: mass inertia matrix.
        )mydelimiter")


        .def("get_non_linearities", [](raisim::ArticulatedSystem &self) {
            VecDyn vec = self.getNonlinearities();
            return convert_vecdyn_to_np(vec);
        }, R"mydelimiter(
        Get the non linearity terms that are present in the dynamic equation of motion:

        .. math:: H(q) \ddot{q} + N(q, \dot{q}) = \tau

        where :math:`N(q, \dot{q})` are the non-linear terms, :math:`H(q)` is the inertia matrix, and :math:`\tau` are
        the forces/torques.

        Returns:
            np.array[float[n]]: non-linearity forces.
        )mydelimiter")


        .def("get_inverse_mass_matrix", [](raisim::ArticulatedSystem &self) {
            MatDyn mat = self.getInverseMassMatrix();
            return convert_matdyn_to_np(mat);
        }, R"mydelimiter(
        Get the inverse of the mass (inertia) matrix, where the inertia matrix appears in the dynamic equation of motion:

        .. math:: H(q) \ddot{q} + N(q, \dot{q}) = \tau

        where :math:`H(q)` is the mass inertia matrix, :math:`N(q, \dot{q})` are the non-linear terms, and
        :math:`\tau` are the forces/torques.

        Returns:
            np.array[float[n,n]]: inverse of the mass inertia matrix.
        )mydelimiter")


        .def("get_composite_com", [](raisim::ArticulatedSystem &self) {
            Vec<3> vec = self.getCompositeCOM();
            return convert_vec_to_np(vec);
        }, R"mydelimiter(
        Get the composite center of mass position (i.e. the center of mass of the whole system).

        Returns:
            np.array[float[3]]: center of mass position.
        )mydelimiter")


        .def("get_composite_inertia", [](raisim::ArticulatedSystem &self) {
            Mat<3, 3> vec = self.getCompositeInertia();
            return convert_mat_to_np(vec);
        }, R"mydelimiter(
        Get the composite moment of inertia of the whole system.

        Returns:
            np.array[float[3, 3]]: moment inertia of the whole system.
        )mydelimiter")


        .def("get_linear_momentum", [](raisim::ArticulatedSystem &self) {
            Vec<3> vec = self.getLinearMomentumInCartesianSpace();
            return convert_vec_to_np(vec);
        }, R"mydelimiter(
        Get the linear momentum of the whole system in Cartesian space.

        Returns:
            np.array[float[3]]: total linear momentum.
        )mydelimiter")


        .def("get_generalized_momentum", [](raisim::ArticulatedSystem &self) {
            VecDyn vec = self.getGeneralizedMomentum();
            return convert_vecdyn_to_np(vec);
        }, R"mydelimiter(
        Get the generalized momentum which is simply the mass matrix multiplied by the generalized velocities:

        .. math:: H(q) \dot{q}

        where :math:`H(q)` is the mass/inertia matrix, and :math:`\dot{q}` are the generalized velocities.

        Returns:
            np.array[float[n]]: generalized momentum.
        )mydelimiter")


        .def("get_kinetic_energy", &raisim::ArticulatedSystem::getKineticEnergy, "Return the total kinetic energy of the whole system.")

        .def("get_potential_energy", [](raisim::ArticulatedSystem &self, py::array_t<double> gravity) {
            Vec<3> g = convert_np_to_vec<3>(gravity);
            return self.getPotentialEnergy(g);
        }, R"mydelimiter(
        Get the system's potential energy due to gravity.

        Args:
            gravity (np.array[float[3]]): gravity vector.

        Returns:
            float: potential energy.
        )mydelimiter",
        py::arg("gravity"))


        .def("get_energy", [](raisim::ArticulatedSystem &self, py::array_t<double> gravity) {
            Vec<3> g = convert_np_to_vec<3>(gravity);
            return self.getEnergy(g);
        }, R"mydelimiter(
        Get the system's total energy.

        Args:
            gravity (np.array[float[3]]): gravity vector.

        Returns:
            float: total energy.
        )mydelimiter",
        py::arg("gravity"))


        .def("print_body_names_in_order", &raisim::ArticulatedSystem::printOutBodyNamesInOrder,
            "Print the moving bodies in the order. Fixed bodies are optimized out.")

        .def("print_frame_names_in_order", &raisim::ArticulatedSystem::printOutFrameNamesInOrder,
            "Print the frames (that are attached to every joint coordinate) in the order.")

        .def("get_world_position", [](raisim::ArticulatedSystem &self, size_t body_idx, py::array_t<double> body_point) {
            Vec<3> pos;
            Vec<3> body_pos = convert_np_to_vec<3>(body_point);
            self.getPosition_W(body_idx, body_pos, pos);
            return convert_vec_to_np(pos);
        }, R"mydelimiter(
        Get the body's position with respect to the world frame.

        Args:
            body_idx (int): body/link index.
            body_point (np.array[float[3]]): position of point on the body expressed in the body frame.

        Returns:
            np.array[float[3]]: position of the body point in the world frame.
        )mydelimiter",
        py::arg("body_idx"), py::arg("body_point"))


        /* CoordinateFrame contains the pose relative to its parent expressed in the parent frame.
         If you want the position expressed in the world frame,
         you have to use this with "void getPosition_W(size_t bodyIdx, const Vec<3> &point_B, Vec<3> &point_W)".
         If you want the orientation expressed in the world frame,
         you have to get the parent body orientation and pre-multiply it by the relative orientation*/
        .def("get_frame_by_name", &raisim::ArticulatedSystem::getFrameByName, R"mydelimiter(
        Get the coordinate frame from its name.

        Args:
            name (str): name of the frame.

        Returns:
            CoordinateFrame: the coordinate frame.
        )mydelimiter",
        py::arg("name"))


        .def("get_frame_by_idx", &raisim::ArticulatedSystem::getFrameByIdx, R"mydelimiter(
        Get the coordinate frame from its index.

        Args:
            idx (int): index of the frame.

        Returns:
            CoordinateFrame: the coordinate frame.
        )mydelimiter",
        py::arg("idx"))


        .def("get_frame_idx_by_name", &raisim::ArticulatedSystem::getFrameIdxByName, R"mydelimiter(
        Get the coordinate frame index from its name.

        Args:
            name (str): name of the frame.

        Returns:
            int: the corresponding index.
        )mydelimiter",
        py::arg("name"))


        .def("get_frames", &raisim::ArticulatedSystem::getFrames, R"mydelimiter(
        Get all the coordinate frames.

        Returns:
            list[CoordinateFrame]: the coordinate frames.
        )mydelimiter")


        /* returns position and orientation in the world frame of a frame defined in the robot description
          Frames are attached to the joint position */
        .def("get_frame_world_position", [](raisim::ArticulatedSystem &self, size_t frame_id) {
            Vec<3> vec;
            self.getFramePosition_W(frame_id, vec);
            return convert_vec_to_np(vec);
        }, R"mydelimiter(
        Get the frame position expressed in the Cartesian world frame.

        Args:
            frame_id (int): frame id.

        Returns:
            np.array[float[3]]: the coordinate frame position in the world space.
        )mydelimiter",
        py::arg("frame_id"))


        .def("get_frame_world_rotation_matrix", [](raisim::ArticulatedSystem &self, size_t frame_id) {
            Mat<3, 3> mat;
            self.getFrameOrientation_W(frame_id, mat);
            return convert_mat_to_np(mat);
        }, R"mydelimiter(
        Get the frame orientation as a rotation matrix expressed in the Cartesian world frame

        Args:
            frame_id (int): frame id.

        Returns:
            np.array[float[3,3]]: the coordinate frame orientation in the world space.
        )mydelimiter",
        py::arg("frame_id"))


        .def("get_frame_world_quaternion", [](raisim::ArticulatedSystem &self, size_t frame_id) {
            Mat<3, 3> mat;
            self.getFrameOrientation_W(frame_id, mat);
            Vec<4> quat;
            rotMatToQuat(mat, quat);
            return convert_vec_to_np(quat);
        }, R"mydelimiter(
        Get the frame orientation as a quaternion ([w,x,y,z]) expressed in the Cartesian world frame.

        Args:
            frame_id (int): frame id.

        Returns:
            np.array[float[3,3]]: the coordinate frame orientation (quaternion) in the world space.
        )mydelimiter",
        py::arg("frame_id"))


        .def("get_frame_linear_velocity", [](raisim::ArticulatedSystem &self, size_t frame_id) {
            Vec<3> vec;
            self.getFrameVelocity_W(frame_id, vec);
            return convert_vec_to_np(vec);
        }, R"mydelimiter(
        Get the frame linear velocity expressed in the Cartesian world frame.

        Args:
            frame_id (int): frame id.

        Returns:
            np.array[float[3]]: the coordinate frame linear velocity in the world space.
        )mydelimiter",
        py::arg("frame_id"))


        .def("get_frame_angular_velocity", [](raisim::ArticulatedSystem &self, size_t frame_id) {
            Vec<3> vec;
            self.getFrameVelocity_W(frame_id, vec);
            return convert_vec_to_np(vec);
        }, R"mydelimiter(
        Get the frame angular velocity expressed in the Cartesian world frame.

        Args:
            frame_id (int): frame id.

        Returns:
            np.array[float[3]]: the coordinate frame angular velocity in the world space.
        )mydelimiter",
        py::arg("frame_id"))


        .def("get_world_position", [](raisim::ArticulatedSystem &self, size_t frame_id) {
            Vec<3> pos;
            self.getPosition_W(frame_id, pos);
            return convert_vec_to_np(pos);
        }, R"mydelimiter(
        Get the joint frame's position with respect to the world frame.

        Args:
            frame_id (int): frame id.

        Returns:
            np.array[float[3]]: position of the joint frame expressed in the world frame.
        )mydelimiter",
        py::arg("frame_id"))


        .def("get_world_rotation_matrix", [](raisim::ArticulatedSystem &self, size_t frame_id) {
            Mat<3, 3> rot;
            self.getFrameOrientation_W(frame_id, rot);
            return convert_mat_to_np(rot);
        }, R"mydelimiter(
        Get the joint frame's orientation (expressed as a rotation matrix) with respect to the world frame.

        Args:
            frame_id (int): frame index.

        Returns:
            np.array[float[3]]: orientation (rotation matrix) of the joint frame expressed in the world frame.
        )mydelimiter",
        py::arg("frame_id"))


        .def("get_world_quaternion", [](raisim::ArticulatedSystem &self, size_t frame_id) {
            Mat<3, 3> rot;
            self.getFrameOrientation_W(frame_id, rot);
            Vec<4> quat;
            rotMatToQuat(rot, quat);
            return convert_vec_to_np(quat);
        }, R"mydelimiter(
        Get the joint frame's orientation (expressed as a quaternion [w,x,y,z]) with respect to the world frame.

        Args:
            frame_id (int): frame index.

        Returns:
            np.array[float[3]]: orientation (quaternion) of the joint frame expressed in the world frame.
        )mydelimiter",
        py::arg("frame_id"))


        .def("get_world_linear_velocity", [](raisim::ArticulatedSystem &self, size_t frame_id) {
            Vec<3> vel;
            self.getVelocity_W(frame_id, vel);
            return convert_vec_to_np(vel);
        }, R"mydelimiter(
        Get the joint frame's linear velocity with respect to the world frame.

        Args:
            frame_id (int): frame id.

        Returns:
            np.array[float[3]]: velocity of the joint frame expressed in the world frame.
        )mydelimiter",
        py::arg("frame_id"))


        .def("get_world_linear_velocity", [](raisim::ArticulatedSystem &self, py::array_t<double> jacobian) {
            SparseJacobian jac;
            MatDyn mat = convert_np_to_matdyn(jacobian);
            jac.v = mat;
            Vec<3> vel;
            self.getVelocity_W(jac, vel);
            return convert_vec_to_np(vel);
        }, R"mydelimiter(
        Return the velocity of the point of the sparse linear jacobian.

        Args:
            sparse_linear_jacobian (np.array[float[3,n]]): sparse linear jacobian.

        Returns:
            np.array[float[3]]: velocity of the point expressed in the world frame.
        )mydelimiter",
        py::arg("sparse_linear_jacobian"))


        .def("get_world_linear_velocity", [](raisim::ArticulatedSystem &self, size_t body_id,
                py::array_t<double> body_pos) {
            Vec<3> pos = convert_np_to_vec<3>(body_pos);
            Vec<3> vel;
            self.getVelocity_W(body_id, pos, vel);
            return convert_vec_to_np(vel);
        }, R"mydelimiter(
        Return the velocity of a point (expressed in the body frame) in the world frame.

        Args:
            body_id (int): body id.
            body_pos (np.array[float[3]]): position of a point on the body frame.

        Returns:
            np.array[float[3]]: velocity of the body expressed in the world frame.
        )mydelimiter",
        py::arg("body_id"), py::arg("body_pos"))


        .def("get_world_angular_velocity", [](raisim::ArticulatedSystem &self, size_t body_id) {
            Vec<3> vel;
            self.getAngularVelocity_W(body_id, vel);
            return convert_vec_to_np(vel);
        }, R"mydelimiter(
        Get the angular velocity of the body with respect to the world frame.

        Args:
            body_id (int): body id.

        Returns:
            np.array[float[3]]: angular velocity of the body expressed in the world frame.
        )mydelimiter",
        py::arg("body_id"))


        /* Jacobians */
        /* Unless otherwise specified, Jacobians map the generalized velocities to the linear velocities of the
        given point expressed in the World frame. */

        .def("get_sparse_linear_jacobian", [](raisim::ArticulatedSystem &self, size_t body_idx, py::array_t<double> point) {
            SparseJacobian jac;
            Vec<3> pos = convert_np_to_vec<3>(point);
            self.getSparseJacobian(body_idx, pos, jac);
            return convert_matdyn_to_np(jac.v);
        }, R"mydelimiter(
        Get the sparse linear jacobian.

        .. math:: v_{lin} = J(q) \dot{q}

        Args:
            body_idx (int): body index.
            world_point (np.array[float[3]]): 3d point expressed in the world frame.

        Returns:
            np.array[float[3, n]]: sparse linear jacobian.
        )mydelimiter",
        py::arg("body_idx"), py::arg("world_point"))


        .def("get_sparse_rotational_jacobian", [](raisim::ArticulatedSystem &self, size_t body_idx) {
            SparseJacobian jac;
            self.getSparseRotationalJacobian(body_idx, jac);
            return convert_matdyn_to_np(jac.v);
        }, R"mydelimiter(
        Get the sparse rotational jacobian.

        .. math:: \omega = J(q) \dot{q}

        Args:
            body_idx (int): body index.

        Returns:
            np.array[float[3, n]]: sparse rotational jacobian.
        )mydelimiter",
        py::arg("body_idx"))


        .def("get_sparse_linear_jacobian_time_derivative", [](raisim::ArticulatedSystem &self, size_t body_idx, py::array_t<double> point) {
            SparseJacobian jac;
            Vec<3> pos = convert_np_to_vec<3>(point);
            self.getSparseJacobianTimeDerivative(body_idx, pos, jac);
            return convert_matdyn_to_np(jac.v);
        }, R"mydelimiter(
        Get the time derivative of the sparse linear jacobian :math:`\dot{J}(q)`.

        Args:
            body_idx (int): body index.
            world_point (np.array[float[3]]): 3d point expressed in the world frame.

        Returns:
            np.array[float[3, n]]: time derivative of the sparse linear jacobian.
        )mydelimiter",
        py::arg("body_idx"), py::arg("world_point"))


        .def("get_sparse_rotational_jacobian_time_derivative", [](raisim::ArticulatedSystem &self, size_t body_idx, py::array_t<double> point) {
            SparseJacobian jac;
            Vec<3> pos = convert_np_to_vec<3>(point);
            self.getSparseRotationalJacobianTimeDerivative(body_idx, pos, jac);
            return convert_matdyn_to_np(jac.v);
        }, R"mydelimiter(
        Get the time derivative of the sparse rotational jacobian :math:`\dot{J}(q)`.

        Args:
            body_idx (int): body index.
            world_point (np.array[float[3]]): 3d point expressed in the world frame.

        Returns:
            np.array[float[3, n]]: time derivative of the sparse rotational jacobian.
        )mydelimiter",
        py::arg("body_idx"), py::arg("world_point"))


        .def("get_dense_linear_jacobian", [](raisim::ArticulatedSystem &self, size_t body_idx, py::array_t<double> point) {
            size_t n = self.getGeneralizedCoordinateDim();
            Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(3, n);
            Vec<3> pos = convert_np_to_vec<3>(point);
            self.getDenseJacobian(body_idx, pos, jac);
            return jac;
        }, R"mydelimiter(
        Get the dense linear jacobian.

        .. math:: v_{lin} = J(q) \dot{q}

        Args:
            body_idx (int): body index.
            world_point (np.array[float[3]]): 3d point expressed in the world frame.

        Returns:
            np.array[float[3, n]]: dense linear jacobian.
        )mydelimiter",
        py::arg("body_idx"), py::arg("world_point"))


        .def("get_dense_rotational_jacobian", [](raisim::ArticulatedSystem &self, size_t body_idx) {
            size_t n = self.getGeneralizedCoordinateDim();
            Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(3, n);
            self.getDenseOrientationalJacobian(body_idx, jac);
            return jac;
        }, R"mydelimiter(
        Get the dense rotational jacobian.

        .. math:: \omega = J(q) \dot{q}

        Args:
            body_idx (int): body index.

        Returns:
            np.array[float[3, n]]: dense rotational jacobian.
        )mydelimiter",
        py::arg("body_idx"))


        .def("get_body_index", &raisim::ArticulatedSystem::getBodyIdx, R"mydelimiter(
        Return the body index associated with the given name.

        Args:
            name (str): body name.

        Returns:
            int: body index.
        )mydelimiter",
        py::arg("name"))


        .def("pre_contact_solver_update1", [](raisim::ArticulatedSystem &self, py::array_t<double> gravity, double dt) {
            Vec<3> g = convert_np_to_vec<3>(gravity);
            self.preContactSolverUpdate1(g, dt);
        }, R"mydelimiter(
        This method:

        1. updates kinematics if necessary (if user didn't call it manually).
        2. computes the mass matrix, nonlinear term, inverse of the mass matrix, and contact jacobians.

        Note that the `integrate` method will automatically call this method.

        Args:
            gravity (np.array[float[3]]): gravity vector.
            dt (float): integration time.
        )mydelimiter",
        py::arg("gravity"), py::arg("dt"))


        .def("get_num_dof", &raisim::ArticulatedSystem::getDOF, R"mydelimiter(
        Return the number of degrees of freedom.

        Returns:
            int: the number of degrees of freedom.
        )mydelimiter")


        .def("get_dof", &raisim::ArticulatedSystem::getDOF, R"mydelimiter(
        Return the number of degrees of freedom.

        Returns:
            int: the number of degrees of freedom.
        )mydelimiter")


        .def("get_generalized_coordinate_dim", &raisim::ArticulatedSystem::getGeneralizedCoordinateDim, R"mydelimiter(
        Return the dimension/size of the generalized coordinates vector.

        Returns:
            int: the number of generalized coordinates.
        )mydelimiter")


        .def("get_body_pose", [](raisim::ArticulatedSystem &self, size_t body_id) {
            Vec<3> pos;
            Mat<3,3> rot;
            self.getBodyPose(body_id, rot, pos);

            Vec<4> quat;
            rotMatToQuat(rot, quat);

            auto position = convert_vec_to_np(pos);
            auto orientation = convert_vec_to_np(quat);
            return position, orientation;
        }, R"mydelimiter(
        Return the body pose (position and orientation (expressed as a quaternion)).

        Args:
            body_id (int): body id.

        Returns:
            np.array[float[3]]: body position.
            np.array[float[4]]: body orientation (expressed as a quaternion [w,x,y,z])
        )mydelimiter",
        py::arg("body_id"))


        .def("get_body_pose1", [](raisim::ArticulatedSystem &self, size_t body_id) {
            Vec<3> pos;
            Mat<3,3> rot;
            self.getBodyPose(body_id, rot, pos);
            auto position = convert_vec_to_np(pos);
            auto orientation = convert_mat_to_np(rot);
            return position, orientation;
        }, R"mydelimiter(
        Return the body pose (position and orientation (expressed as a rotation matrix)).

        Args:
            body_id (int): body id.

        Returns:
            np.array[float[3]]: body position.
            np.array[float[3,3]]: body orientation (expressed as a rotation matrix)
        )mydelimiter",
        py::arg("body_id"))


        /* The following 5 methods can be used to directly modify dynamic/kinematic properties of the robot.
         They are made for dynamic randomization. Use them with caution since they will change the
         the model permanently. After you change the dynamic properties, call "void updateMassInfo()" to update
         some precomputed dynamic properties.
         returns the reference to joint position relative to its parent, expressed in the parent frame. */

        .def("get_joint_cartesian_positions", [](raisim::ArticulatedSystem &self) {
            std::vector<raisim::Vec<3>>& positions = self.getJointPos_P();

            py::list list;
            for (auto pos : positions)
                list.append(convert_vec_to_np(pos));

            return list;
        }, R"mydelimiter(
        Return the joint Cartesian positions relative to their parent frame.

        Returns:
            list[np.array[float[3]]]: joint cartesian positions (relative to their parent frame).
        )mydelimiter")

        .def("set_joint_cartesian_positions", [](raisim::ArticulatedSystem &self, py::list &list) {
            // get references to joint cartesian positions
            std::vector<raisim::Vec<3>>& positions = self.getJointPos_P();

            // check list and vector sizes
            if (list.size() != positions.size()) {
                std::ostringstream s;
                s << "error: expecting the given list to have the same size as the number of joint positions: "
                    << list.size() << " != " << positions.size() << ".";
                throw std::domain_error(s.str());
            }

            // copy each element from the list
            for (size_t i=0; i<list.size(); i++) {
                py::array_t<double> e = list[i].cast<py::array_t<double>>();
                positions[i] = convert_np_to_vec<3>(e);
            }

            // update mass info (this is more pythonic)
            self.updateMassInfo();
        }, R"mydelimiter(
        Set the joint Cartesian positions relative to their parent frame.

        Args:
            positions (list[np.array[float[3]]]): joint cartesian positions (relative to their parent frame).
        )mydelimiter",
        py::arg("positions"))

        .def_property("joint_cartesian_positions",
            [](raisim::ArticulatedSystem &self) {  // getter
                std::vector<raisim::Vec<3>> positions = self.getJointPos_P();

                py::list list;
                for (auto pos : positions)
                    list.append(convert_vec_to_np(pos));

                return list;
            },
            [](raisim::ArticulatedSystem &self, py::list &list) { // setter
                // get references to joint cartesian positions
                std::vector<raisim::Vec<3>>& positions = self.getJointPos_P();

                // check list and vector sizes
                if (list.size() != positions.size()) {
                    std::ostringstream s;
                    s << "error: expecting the given list to have the same size as the number of joint positions: "
                        << list.size() << " != " << positions.size() << ".";
                    throw std::domain_error(s.str());
                }

                // copy each element from the list
                for (size_t i=0; i<list.size(); i++) {
                    py::array_t<double> e = list[i].cast<py::array_t<double>>();
                    positions[i] = convert_np_to_vec<3>(e);
                }

                // update mass info (this is more pythonic)
                self.updateMassInfo();
            })



        .def("get_masses", py::overload_cast<>(&raisim::ArticulatedSystem::getMass), R"mydelimiter(
        Return the body/link masses.

        Returns:
            list[double]: body masses.
        )mydelimiter")

        .def("set_masses", [](raisim::ArticulatedSystem &self, std::vector<double> &list) {
            // get references to masses
            std::vector<double>& masses = self.getMass();

            // check list and vector sizes
            if (list.size() != masses.size()) {
                std::ostringstream s;
                s << "error: expecting the given list to have the same size as the number of masses: "
                    << list.size() << " != " << masses.size() << ".";
                throw std::domain_error(s.str());
            }

            // copy each element
            for (size_t i=0; i<list.size(); i++) {
                masses[i] = list[i];
            }

            // update mass info (this is more pythonic)
            self.updateMassInfo();
        }, R"mydelimiter(
        Set the body/link masses.

        Args:
            masses (list[double]): body masses.
        )mydelimiter",
        py::arg("masses"))

        .def_property("masses",
            py::overload_cast<>(&raisim::ArticulatedSystem::getMass), // getter
            [](raisim::ArticulatedSystem &self, std::vector<double> &list) { // setter
                // get references to masses
                std::vector<double>& masses = self.getMass();

                // check list and vector sizes
                if (list.size() != masses.size()) {
                    std::ostringstream s;
                    s << "error: expecting the given list to have the same size as the number of masses: "
                        << list.size() << " != " << masses.size() << ".";
                    throw std::domain_error(s.str());
                }

                // copy each element
                for (size_t i=0; i<list.size(); i++) {
                    masses[i] = list[i];
                }

                // update mass info (this is more pythonic)
                self.updateMassInfo();
            })


        .def("get_inertias", [](raisim::ArticulatedSystem &self) {
            std::vector<raisim::Mat<3, 3> >& inertia = self.getInertia();

            py::list list;
            for (auto I : inertia)
                list.append(convert_mat_to_np(I));

            return list;
        }, R"mydelimiter(
        Return the inertias (one for each body).

        Returns:
            list[np.array[float[3,3]]]: inertias.
        )mydelimiter")

        .def("set_inertias", [](raisim::ArticulatedSystem &self, py::list &list) {
            // get references to inertias
            std::vector<raisim::Mat<3, 3> >& inertia = self.getInertia();

            // check list and vector sizes
            if (list.size() != inertia.size()) {
                std::ostringstream s;
                s << "error: expecting the given list to have the same size as the number of inertia: "
                    << list.size() << " != " << inertia.size() << ".";
                throw std::domain_error(s.str());
            }

            // copy each element from the list
            for (size_t i=0; i<list.size(); i++) {
                py::array_t<double> e = list[i].cast<py::array_t<double>>();
                inertia[i] = convert_np_to_mat<3, 3>(e);
            }

            // update mass info (this is more pythonic)
            self.updateMassInfo();
        }, R"mydelimiter(
        Set the inertias (one for each body).

        Args:
            inertias (list[np.array[float[3,3]]]): body inertias.
        )mydelimiter",
        py::arg("inertias"))

        .def_property("inertias",
            [](raisim::ArticulatedSystem &self) {  // getter
                std::vector<raisim::Mat<3, 3> >& inertia = self.getInertia();

                py::list list;
                for (auto I : inertia)
                    list.append(convert_mat_to_np(I));

                return list;
            },
            [](raisim::ArticulatedSystem &self, py::list &list) { // setter
                // get references to inertias
                std::vector<raisim::Mat<3, 3> >& inertia = self.getInertia();

                // check list and vector sizes
                if (list.size() != inertia.size()) {
                    std::ostringstream s;
                    s << "error: expecting the given list to have the same size as the number of inertia: "
                        << list.size() << " != " << inertia.size() << ".";
                    throw std::domain_error(s.str());
                }

                // copy each element from the list
                for (size_t i=0; i<list.size(); i++) {
                    py::array_t<double> e = list[i].cast<py::array_t<double>>();
                    inertia[i] = convert_np_to_mat<3, 3>(e);
                }

                // update mass info (this is more pythonic)
                self.updateMassInfo();
            })



        .def("get_link_coms", [](raisim::ArticulatedSystem &self) {
            std::vector<raisim::Vec<3>> positions = self.getLinkCOM();

            py::list list;
            for (auto pos : positions)
                list.append(convert_vec_to_np(pos));

            return list;
        }, R"mydelimiter(
        Return the center of mass of each link (expressed in their body frame).

        Returns:
            list[np.array[float[3]]]: center of mass of each link (expressed in the body frame).
        )mydelimiter")

        .def("set_link_coms", [](raisim::ArticulatedSystem &self, py::list &list) {
            // get references to joint cartesian positions
            std::vector<raisim::Vec<3>>& positions = self.getLinkCOM();

            // check list and vector sizes
            if (list.size() != positions.size()) {
                std::ostringstream s;
                s << "error: expecting the given list to have the same size as the number of CoMs: "
                    << list.size() << " != " << positions.size() << ".";
                throw std::domain_error(s.str());
             }

            // copy each element from the list
            for (size_t i=0; i<list.size(); i++) {
                py::array_t<double> e = list[i].cast<py::array_t<double>>();
                positions[i] = convert_np_to_vec<3>(e);
            }

            // update mass info (this is more pythonic)
            self.updateMassInfo();
        }, R"mydelimiter(
        Set the center of mass of each link (expressed in their body frame).

        Args:
            coms (list[np.array[float[3]]]): center of mass of each link (expressed in the body frame).
        )mydelimiter",
        py::arg("coms"))

        .def_property("link_coms",
            [](raisim::ArticulatedSystem &self) {  // getter
                std::vector<raisim::Vec<3>>& positions = self.getLinkCOM();

                py::list list;
                for (auto pos : positions)
                    list.append(convert_vec_to_np(pos));

                return list;
            },
            [](raisim::ArticulatedSystem &self, py::list &list) { // setter
                // get references to joint cartesian positions
                std::vector<raisim::Vec<3>>& positions = self.getLinkCOM();

                // check list and vector sizes
                if (list.size() != positions.size()) {
                    std::ostringstream s;
                    s << "error: expecting the given list to have the same size as the number of CoMs: "
                        << list.size() << " != " << positions.size() << ".";
                    throw std::domain_error(s.str());
                 }

                // copy each element from the list
                for (size_t i=0; i<list.size(); i++) {
                    py::array_t<double> e = list[i].cast<py::array_t<double>>();
                    positions[i] = convert_np_to_vec<3>(e);
                }

                // update mass info (this is more pythonic)
                self.updateMassInfo();
            })



        .def("get_collision_bodies", [](raisim::ArticulatedSystem &self) {
            return self.getCollisionBodies();
        }, R"mydelimiter(
        Return the collision bodies.

        Returns:
            list[CollisionDefinition]: collision bodies.
        )mydelimiter")
        .def("set_collision_bodies", [](raisim::ArticulatedSystem &self, raisim::CollisionSet &list) {
            // get references to collision bodies
            std::vector<CollisionDefinition> collisions = self.getCollisionBodies();

            // check list and vector sizes
            if (list.size() != collisions.size()) {
                std::ostringstream s;
                s << "error: expecting the given list to have the same size as the number of collision bodies: "
                     << list.size() << " != " << collisions.size() << ".";
                throw std::domain_error(s.str());
            }

            // copy each element from the list
            for (size_t i=0; i<list.size(); i++) {
                collisions[i] = list[i];
            }

            // update mass info (this is more pythonic)
            self.updateMassInfo();
        }, R"mydelimiter(
        Set the collision bodies.

        Args:
            collisions (list[CollisionDefinition]): collision bodies.
        )mydelimiter",
        py::arg("collisions"))

        .def_property("collision_bodies",
            [](raisim::ArticulatedSystem &self) {  // getter
                return self.getCollisionBodies();
            },
            [](raisim::ArticulatedSystem &self, py::list list) { // setter
                // get references to collision bodies
                std::vector<CollisionDefinition> collisions = self.getCollisionBodies();

                // check list and vector sizes
                if (list.size() != collisions.size()) {
                    std::ostringstream s;
                    s << "error: expecting the given list to have the same size as the number of collision bodies: "
                         << list.size() << " != " << collisions.size() << ".";
                    throw std::domain_error(s.str());
                }

                // copy each element from the list
                for (size_t i=0; i<list.size(); i++) {
                    collisions[i] = list[i].cast<CollisionDefinition>();
                }

                // update mass info (this is more pythonic)
                self.updateMassInfo();
            })


        .def("get_collision_body", &raisim::ArticulatedSystem::getCollisionBody, R"mydelimiter(
        Return the collision body associated with the given name.

        Args:
            name (str): collision body name.

        Returns:
            CollisionDefinition: collision body.
        )mydelimiter",
        py::arg("name"))

        .def("set_collision_body", [](raisim::ArticulatedSystem &self, const std::string& name,
                CollisionDefinition collision) {

            CollisionDefinition col = self.getCollisionBody(name);

            // copy the values
            col.rotOffset = collision.rotOffset;
            col.posOffset = collision.posOffset;
            col.localIdx = collision.localIdx;
            col.colObj = collision.colObj;
            col.name = collision.name;
        }, R"mydelimiter(
        Set the collision body to the given name.

        Args:
            name (str): collision body name.
            collision (CollisionDefinition): collision definition.
        )mydelimiter",
        py::arg("name"), py::arg("collision"))



        // this is automatically done when you use properties so you don't have to call it (more pythonic)
        .def("update_mass_info", &raisim::ArticulatedSystem::updateMassInfo,
            "Update the mass information. This function must be called after we change the dynamic parameters.")


        .def("get_mass", py::overload_cast<size_t>(&raisim::ArticulatedSystem::getMass), R"mydelimiter(
        Get the mass of the link.

        Args:
            link_idx (int): link index.

        Returns:
            float: mass value.
        )mydelimiter",
        py::arg("link_idx"))

        .def("set_mass", &raisim::ArticulatedSystem::setMass, R"mydelimiter(
        Set the mass of the link.

        Args:
            link_idx (int): link index.
            value (float): mass value.
        )mydelimiter",
        py::arg("link_idx"), py::arg("value"))


        .def("get_total_mass", &raisim::ArticulatedSystem::getTotalMass, R"mydelimiter(
        Get the total mass of the system.

        Returns:
            float: total mass value.
        )mydelimiter")


        .def("set_external_force", [](raisim::ArticulatedSystem &self, size_t local_idx, py::array_t<double> force) {
	        Vec<3> f = convert_np_to_vec<3>(force);
	        self.setExternalForce(local_idx, f);
	    }, R"mydelimiter(
	    Set the external force on the body.

	    Args:
	        local_idx (int): local/link index.
	        force (np.array[float[3]]): force vector.
	    )mydelimiter",
	    py::arg("local_idx"), py::arg("force"))


	    .def("set_external_torque", [](raisim::ArticulatedSystem &self, size_t local_idx, py::array_t<double> torque) {
	        Vec<3> t = convert_np_to_vec<3>(torque);
	        self.setExternalTorque(local_idx, t);
	    }, R"mydelimiter(
	    Set the external force on the body.

	    Args:
	        local_idx (int): local/link index.
	        force (np.array[float[3]]): force vector.
	    )mydelimiter",
	    py::arg("local_idx"), py::arg("torque"))


	    .def("set_external_force", [](raisim::ArticulatedSystem &self, size_t local_idx,
	            raisim::ArticulatedSystem::Frame force_frame, py::array_t<double> force,
	            raisim::ArticulatedSystem::Frame pos_frame, py::array_t<double> position) {
	        Vec<3> f = convert_np_to_vec<3>(force);
	        self.setExternalForce(local_idx, f);
	    }, R"mydelimiter(
	    Set the external force on the specified point on the body.

	    Args:
	        local_idx (int): local/link index.
	        force_frame (Frame): frame in which the force vector is expressed in, select between {WORLD_FRAME,
	            BODY_FRAME, PARENT_FRAME}.
	        force (np.array[float[3]]): force vector.
	        pos_frame (Frame): frame in which the position vector is expressed in, select between {WORLD_FRAME,
	            BODY_FRAME, PARENT_FRAME}.
	        position (np.array[float[3]]): position vector.

	    )mydelimiter",
	    py::arg("local_idx"), py::arg("force_frame"), py::arg("force"), py::arg("pos_frame"), py::arg("position"))


        .def("set_control_mode", &raisim::ArticulatedSystem::setControlMode, R"mydelimiter(
	    Set the control mode.

	    Args:
	        mode (ControlMode.Type): control mode type, select between {FORCE_AND_TORQUE, PD_PLUS_FEEDFORWARD_TORQUE,
	            VELOCITY_PLUS_FEEDFORWARD_TORQUE}
	    )mydelimiter",
	    py::arg("mode"))

        .def("get_control_mode", &raisim::ArticulatedSystem::getControlMode, R"mydelimiter(
	    Get the control mode.

	    Returns:
	        ControlMode.Type: control mode type which is one of {FORCE_AND_TORQUE, PD_PLUS_FEEDFORWARD_TORQUE,
	            VELOCITY_PLUS_FEEDFORWARD_TORQUE}
	    )mydelimiter")


        /* set PD targets. It is effective only in the control mode "PD_PLUS_FEEDFORWARD_TORQUE". set any arbitrary
        number for unactuated degrees of freedom */

        .def("set_pd_targets", py::overload_cast<const Eigen::VectorXd &, const Eigen::VectorXd &>(&raisim::ArticulatedSystem::setPdTarget), R"mydelimiter(
	    Set the PD targets. It is effective only in the control mode 'PD_PLUS_FEEDFORWARD_TORQUE'. Set any arbitrary
        number for unactuated degrees of freedom.

	    Args:
	        pos_targets (np.array[float[n]]): position targets.
	        vel_targets (np.array[float[n]]): velocity targets.
	    )mydelimiter",
	    py::arg("pos_targets"), py::arg("vel_targets"))


        .def("set_pd_gains", py::overload_cast<const Eigen::VectorXd &, const Eigen::VectorXd &>(&raisim::ArticulatedSystem::setPdGains), R"mydelimiter(
	    Set the PD gains. It is effective only in the control mode 'PD_PLUS_FEEDFORWARD_TORQUE'. Set any arbitrary
        number for unactuated degrees of freedom.

	    Args:
	        p_gains (np.array[float[n]]): P gains.
	        d_gains (np.array[float[n]]): D gains.
	    )mydelimiter",
	    py::arg("p_gains"), py::arg("d_gains"))


	    .def("set_joint_dampings", py::overload_cast<const Eigen::VectorXd &>(&raisim::ArticulatedSystem::setJointDamping), R"mydelimiter(
	    Set the joint dampings (passive elements at the joints).

	    Args:
	        dampings (np.array[float[n]]): joint damping coefficients.
	    )mydelimiter",
	    py::arg("dampings"))


        .def("compute_sparse_inverse_mass_matrix", [](raisim::ArticulatedSystem &self, py::array_t<double> mass) {
            MatDyn M = convert_np_to_matdyn(mass);
            MatDyn Minv;
            self.computeSparseInverse(M, Minv);
            return convert_matdyn_to_np(Minv);
        }, R"mydelimiter(
	    This computes the inverse mass matrix given the mass matrix. The return type is dense. It exploits the
	    sparsity of the mass matrix to efficiently perform the computation.

	    Args:
	        mass_matrix (np.array[float[n,n]]): mass matrix.

	    Returns:
	        np.array(float[n,n]): dense inverse matrix.
	    )mydelimiter",
	    py::arg("mass_matrix"))


//	    .def("mass_matrix_vector_multiplication", [](raisim::ArticulatedSystem &self, py::array_t<double> vector) {
//            VecDyn vec1 = convert_np_to_vecdyn(vector);
//            VecDyn vec;
//            self.massMatrixVecMul(vec1, vec);
//            return convert_vecdyn_to_np(vec);
//        }, R"mydelimiter(
//	    This method exploits the sparsity of the mass matrix. If the mass matrix is nearly dense, it will be slower
//	    than your ordinary matrix multiplication which is probably vectorized.
//
//	    Args:
//	        vector (np.array[float[n]]): vector to be multiplied by the mass matrix.
//
//	    Returns:
//	        np.array(float[n]): resulting vector.
//	    )mydelimiter",
//	    py::arg("vector"))


	    .def("ignore_collision_between", &raisim::ArticulatedSystem::ignoreCollisionBetween, R"mydelimiter(
	    Ignore collision between the 2 specified bodies.

	    Args:
	        body_idx1 (int): first body index.
	        body_idx2 (int): second body index.
	    )mydelimiter",
	    py::arg("body_idx1"), py::arg("body_idx2"))


	    .def("get_options", &raisim::ArticulatedSystem::getOptions, R"mydelimiter(
	    Return the options associated with the articulated system.

	    Returns:
	        ArticulatedSystemOption: options for the articulated system.
	    )mydelimiter")


	    .def("get_body_names", &raisim::ArticulatedSystem::getBodyNames, R"mydelimiter(
	    Return the body names.

	    Returns:
	        list[str]: list of body names.
	    )mydelimiter")


        .def("get_visual_objects", &raisim::ArticulatedSystem::getVisOb, R"mydelimiter(
	    Get the visual objects.

	    Returns:
	        list[VisObject]: list of visual objects.
	    )mydelimiter")


	    .def("get_visual_collision_objects", &raisim::ArticulatedSystem::getVisColOb, R"mydelimiter(
	    Get the visual collision objects.

	    Returns:
	        list[VisObject]: list of visual collision objects.
	    )mydelimiter")


	    .def("get_visual_object_pose", [](raisim::ArticulatedSystem &self, size_t body_idx) {
	        Vec<3> pos;
            Mat<3,3> rot;
            self.getVisObPose(body_idx, rot, pos);

            Vec<4> quat;
            rotMatToQuat(rot, quat);

            auto position = convert_vec_to_np(pos);
            auto orientation = convert_vec_to_np(quat);
            return position, orientation;
	    }, R"mydelimiter(
	    Get the visual object pose (where the orientation is expressed as a quaternion).

	    Args:
	        body_idx (int): body index.

	    Returns:
	        np.array[float[3]]: visual object position.
	        np.array[float[4]]: visual object orientation (expressed as a quaternion [w,x,y,z]).
	    )mydelimiter")

	    .def("get_visual_object_pose1", [](raisim::ArticulatedSystem &self, size_t body_idx) {
	        Vec<3> pos;
            Mat<3,3> rot;
            self.getVisObPose(body_idx, rot, pos);
            auto position = convert_vec_to_np(pos);
            auto orientation = convert_mat_to_np(rot);
            return position, orientation;
	    }, R"mydelimiter(
	    Get the visual object pose (where the orientation is expressed as a rotation matrix).

	    Args:
	        body_idx (int): body index.

	    Returns:
	        np.array[float[3]]: visual object position.
	        np.array[float[3,3]]: visual object orientation (expressed as a rotation matrix).
	    )mydelimiter")


	    .def("get_visual_collision_object_pose", [](raisim::ArticulatedSystem &self, size_t body_idx) {
	        Vec<3> pos;
            Mat<3,3> rot;
            self.getVisColObPose(body_idx, rot, pos);

            Vec<4> quat;
            rotMatToQuat(rot, quat);

            auto position = convert_vec_to_np(pos);
            auto orientation = convert_vec_to_np(quat);
            return position, orientation;
	    }, R"mydelimiter(
	    Get the visual collision object pose (where the orientation is expressed as a quaternion).

	    Args:
	        body_idx (int): body index.

	    Returns:
	        np.array[float[3]]: visual object position.
	        np.array[float[4]]: visual object orientation (expressed as a quaternion [w,x,y,z]).
	    )mydelimiter")

	    .def("get_visual_collision_object_pose1", [](raisim::ArticulatedSystem &self, size_t body_idx) {
	        Vec<3> pos;
            Mat<3,3> rot;
            self.getVisColObPose(body_idx, rot, pos);
            auto position = convert_vec_to_np(pos);
            auto orientation = convert_mat_to_np(rot);
            return position, orientation;
	    }, R"mydelimiter(
	    Get the visual collision object pose (where the orientation is expressed as a rotation matrix).

	    Args:
	        body_idx (int): body index.

	    Returns:
	        np.array[float[3]]: visual object position.
	        np.array[float[3,3]]: visual object orientation (expressed as a rotation matrix).
	    )mydelimiter")


        .def("get_resource_directory", &raisim::ArticulatedSystem::getResourceDir, R"mydelimiter(
	    Get the robot resource directory.

	    Returns:
	        str: robot resource directory.
	    )mydelimiter")


	    .def("get_robot_description_filename", &raisim::ArticulatedSystem::getRobotDescriptionfFileName, R"mydelimiter(
	    Get the robot description filename (e.g. path to the URDF).

	    Returns:
	        str: robot description filename.
	    )mydelimiter")


	    .def("get_robot_description_directory_name", &raisim::ArticulatedSystem::getRobotDescriptionfTopDirName, R"mydelimiter(
	    Get the robot description top directory name.

	    Returns:
	        str: robot description top directory name.
	    )mydelimiter")


        /* change the base position and orientation of the base. */
        .def("set_base_position", [](raisim::ArticulatedSystem &self, py::array_t<double> position) {
	        Vec<3> pos = convert_np_to_vec<3>(position);
            self.setBasePos(pos);
	    }, R"mydelimiter(
	    Set the base position.

	    Args:
	        position (np.array[float[3]]): new base position.
	    )mydelimiter",
	    py::arg("position"))

	    .def("set_base_orientation", [](raisim::ArticulatedSystem &self, py::array_t<double> orientation) {
	        Mat<3,3> rot;
	        if (orientation.size() == 3) { // rpy angles
	            Vec<3> rpy = convert_np_to_vec<3>(orientation);
                rpyToRotMat_intrinsic(rpy, rot);
	        } else if (orientation.size() == 4) {
	            Vec<4> quat = convert_np_to_vec<4>(orientation);
                quatToRotMat(quat, rot);
	        } else if (orientation.size() == 9) {
	            rot = convert_np_to_mat<3,3>(orientation);
	        } else {
                std::ostringstream s;
                s << "error: expecting the given orientation to have a size of 3 (RPY), 4 (quaternion), or 9 " <<
                    "(rotation matrix), but got instead a size of " << orientation.size() << ".";
                throw std::domain_error(s.str());
	        }
	        self.setBaseOrientation(rot);
	    }, R"mydelimiter(
	    Set the base orientation.

	    Args:
	        orientation (np.array[float[3]], np.array[float[4]], np.array[float[3,3]]): new base orientation.
	    )mydelimiter",
	    py::arg("orientation"))


	    .def("set_actuation_limits", &raisim::ArticulatedSystem::setActuationLimits, R"mydelimiter(
	    Set the upper and lower limits in actuation forces.

	    Args:
	        upper (np.array[float[n]]): upper limits.
	        lower (np.array[float[n]]): lower limits.
	    )mydelimiter",
	    py::arg("upper"), py::arg("lower"))


	    .def("set_collision_object_shape_parameters", &raisim::ArticulatedSystem::setCollisionObjectShapeParameters, R"mydelimiter(
	    Set the collision object shape parameters.

	    Args:
	        id_ (int): collision object id.
	        parameters (list[float]): parameters.
	    )mydelimiter",
	    py::arg("id_"), py::arg("parameters"))


        .def("set_collision_object_position_offset", [](raisim::ArticulatedSystem &self, size_t id_, py::array_t<double> &position) {
            Vec<3> pos = convert_np_to_vec<3>(position);
            self.setCollisionObjectPositionOffset(id_, pos);
        }, R"mydelimiter(
	    Set the collision object position offset.

	    Args:
	        id_ (int): collision object id.
	        position (np.array[float[3]]): position offset.
	    )mydelimiter",
	    py::arg("id_"), py::arg("position"))


	    .def("set_collision_object_position_offset", [](raisim::ArticulatedSystem &self, size_t id_, py::array_t<double> &orientation) {
            Mat<3,3> rot;
	        if (orientation.size() == 3) { // rpy angles
	            Vec<3> rpy = convert_np_to_vec<3>(orientation);
                rpyToRotMat_intrinsic(rpy, rot);
	        } else if (orientation.size() == 4) {
	            Vec<4> quat = convert_np_to_vec<4>(orientation);
                quatToRotMat(quat, rot);
	        } else if (orientation.size() == 9) {
	            rot = convert_np_to_mat<3,3>(orientation);
	        } else {
                std::ostringstream s;
                s << "error: expecting the given orientation to have a size of 3 (RPY), 4 (quaternion), or 9 " <<
                    "(rotation matrix), but got instead a size of " << orientation.size() << ".";
                throw std::domain_error(s.str());
	        }
	        self.setCollisionObjectOrientationOffset(id_, rot);
        }, R"mydelimiter(
	    Set the collision object orientation offset.

	    Args:
	        id_ (int): collision object id.
	        orientation (np.array[float[3]], np.array[float[4]], np.array[float[3,3]]): orientation offset.
	    )mydelimiter",
	    py::arg("id_"), py::arg("orientation"));


    // aliases
    system.attr("get_base_orientation") = system.attr("get_base_quaternion");
    system.attr("get_frame_world_orientation") = system.attr("get_frame_world_quaternion");
    system.attr("get_world_orientation") = system.attr("get_world_quaternion");

}