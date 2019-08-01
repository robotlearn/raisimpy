/**
 * Python wrappers for raisimOgre using pybind11.
 *
 * Copyright (c) 2019, Torus Knot Software Ltd (Ogre - C++), jhwangbo (RaisimOgre - C++),
 *                      Brian Delhaisse <briandelhaisse@gmail.com> (Python wrappers)
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

#include <iostream>

// include Ogre related headers
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"

// include Raisim Ogre related headers
#include "raisim/math.hpp"   // contains the definitions of Vec, Mat, etc.
#include "raisim/interfaceClasses.hpp"
#include "raisim/OgreVis.hpp"
//#include "visualizer/helper.hpp"
//#include "visualizer/guiState.hpp"
#include "visualizer/raisimBasicImguiPanel.hpp"   // for `imguiSetupCallback` and  `imguiRenderCallBack`
#include "visualizer/raisimKeyboardCallback.hpp"  // for `raisimKeyboardCallback`
#include "visualizer/visSetupCallback.hpp"        // for `setupCallback`

// include headers that allows to convert between raisim::Vec, raisim::Mat, raisim::VecDyn, raisim::Mat to np.arrays.
#include "converter.hpp"

namespace py = pybind11;
using namespace raisim;


/// \brief: Visualizer class which inherits from raisim::OgreVis. It adds other few functionalities that were missing.
//class Visualizer : public raisim::OgreVis {
//
//  public:
//
//    using MouseButtonCallback = std::function<bool(const MouseButtonEvent &)>;
//    using MouseMotionCallback = std::function<bool(const MouseMotionEvent &)>;
//    using MouseWheelCallback = std::function<bool(const MouseWheelEvent &)>;
//
//    /** set mouse callback. This callback is called for every mouse event */
//    void setMouseButtonCallback(MouseButtonCallback callback) { mouseButtonCallback_ = callback; }
//    void setMouseMotionCallback(MouseMotionCallback callback) { mouseMotionCallback_ = callback; }
//    void setMouseWheelCallback(MouseWheelCallback callback) { mouseWheelCallback_ = callback; }
//
//  private:
//
//    MouseButtonCallback mouseButtonCallback_ = nullptr;
//    MouseMotionCallback mouseMotionCallback_ = nullptr;
//    MouseWheelCallback mouseWheelCallback_ = nullptr;
//};


void init_visualizer(py::module &m) {

    // create submodule
    py::module visualizer_module = m.def_submodule("visualizer", "RaiSim visualizer submodule.");


    /********/
    /* Ogre */
    /********/

    py::class_<Ogre::SceneNode>(visualizer_module, "SceneNode",
        "Ogre SceneNode: a SceneNode is a logical element of the scene graph hierarchy, which is used to organise "
        "objects in a scene.")
        .def("show_bounding_box", &Ogre::SceneNode::showBoundingBox, R"mydelimiter(
            Show or hide the bounding box of the node

            Args:
                show (bool): if True, show the bounding box.
            )mydelimiter",
            py::arg("show"))
        .def("hide_bounding_box", &Ogre::SceneNode::hideBoundingBox, R"mydelimiter(
            Show or hide the bounding box of the node

            Args:
                hide (bool): if True, hide the bounding box.
            )mydelimiter",
            py::arg("hide"))
        .def("set_visible", &Ogre::SceneNode::setVisible, R"mydelimiter(
            Makes all objects attached to this node become visible / invisible.

            Args:
                visible (bool): whether the objects are to be made visible or invisible.
                cascade (bool): If true, this setting cascades into child nodes too.
            )mydelimiter",
            py::arg("visible"), py::arg("cascade")=true)

        .def("get_position", [](Ogre::SceneNode &self) {
            Ogre::Vector3 vec = self.getPosition();
            return convert_ogre_vec3_to_np(vec);
        }, R"mydelimiter(
            Get the node position.

            Returns:
                np.array[float[3]]: position vector.
            )mydelimiter")
        .def("set_position", [](Ogre::SceneNode &self, py::array_t<double> &pos) {
            Ogre::Vector3 vec = convert_np_to_ogre_vec3(pos);
            self.setPosition(vec);
        }, R"mydelimiter(
            Set the node position.

            Args:
                pos (np.array[float[3]]): position vector.
            )mydelimiter",
            py::arg("pos"))

        .def("get_orientation", [](Ogre::SceneNode &self) {
            Ogre::Quaternion quat = self.getOrientation();
            return convert_ogre_quat_to_np(quat);
        }, R"mydelimiter(
            Get the node orientation (expressed as a quaternion).

            Returns:
                np.array[float[4]]: quaternion [w,x,y,z].
            )mydelimiter")
        .def("set_orientation", [](Ogre::SceneNode &self, py::array_t<double> &quat) {
            Ogre::Quaternion q = convert_np_to_ogre_quat(quat);
            self.setOrientation(q);
        }, R"mydelimiter(
            Set the node orientation (expressed as a quaternion).

            Args:
                quat (np.array[float[4]]): quaternion [w,x,y,z].
            )mydelimiter",
            py::arg("quat"))

        .def("get_scale", [](Ogre::SceneNode &self) {
            Ogre::Vector3 vec = self.getScale();
            return convert_ogre_vec3_to_np(vec);
        }, R"mydelimiter(
            Get the node scale.

            Returns:
                np.array[float[3]]: scale vector.
            )mydelimiter")
        .def("set_scale", [](Ogre::SceneNode &self, py::array_t<double> &scale) {
            Ogre::Vector3 vec = convert_np_to_ogre_vec3(scale);
            self.setScale(vec);
        }, R"mydelimiter(
            Set the node scale.

            Args:
                scale (np.array[float[3]]): scale vector.
            )mydelimiter",
            py::arg("scale"));
        // add other methods if necessary from:
        // - https://github.com/OGRECave/ogre/blob/master/OgreMain/include/OgreSceneNode.h
        // - https://github.com/OGRECave/ogre/blob/master/OgreMain/include/OgreNode.h


    py::class_<Ogre::Light> light(visualizer_module, "Light",
        "Ogre Light: Lights are added to the scene like any other object. They contain various "
        "parameters like type, position, attenuation (how light intensity fades with distance), colour etc.");

    py::enum_<Ogre::Light::LightTypes>(light, "LightTypes")
        .value("LT_POINT", Ogre::Light::LightTypes::LT_POINT, "Point light sources give off light equally in all directions, so require only position not direction")
        .value("LT_DIRECTIONAL", Ogre::Light::LightTypes::LT_DIRECTIONAL, "Directional lights simulate parallel light beams from a distant source, hence have direction but no position")
        .value("LT_SPOTLIGHT", Ogre::Light::LightTypes::LT_SPOTLIGHT, "Spotlights simulate a cone of light from a source so require position and direction, plus extra values for falloff");

    light.def(py::init<>(), "Default light constructor")
        .def(py::init<const std::string &>(), "Normal constructor. Should not be called directly, but rather the SceneManager::createLight method should be used.");
        // add other methods if necessary from https://github.com/OGRECave/ogre/blob/master/OgreMain/include/OgreLight.h


    py::class_<Ogre::Viewport>(visualizer_module, "Viewport", "Ogre viewport: An abstraction of a viewport, i.e. a rendering region on a render target.");

    // TODO: you can also check the Ogre::Camera class from https://ogrecave.github.io/ogre/api/1.10/class_ogre_1_1_camera.html
    // You can get the projection and view matrix for instance.


    /*****************/
    /* GraphicObject */
    /*****************/

    py::class_<raisim::GraphicObject>(visualizer_module, "GraphicObject", "Graphic object represents the underlying object.")
        .def(py::init<>(), "Instantiate the Graphic Object by setting its orientation, scale, and offset position.")
        .def_property("pos_offset",
            [](raisim::GraphicObject &self) {  // getter
                return convert_vec_to_np(self.offset);
            }, [](raisim::GraphicObject &self, py::array_t<double> array) {  // setter
                Vec<3> pos = convert_np_to_vec<3>(array);
                self.offset = pos;
            })
        .def_property("scale",
            [](raisim::GraphicObject &self) {  // getter
                return convert_vec_to_np(self.scale);
            }, [](raisim::GraphicObject &self, py::array_t<double> array) {  // setter
                Vec<3> scale = convert_np_to_vec<3>(array);
                self.scale = scale;
            })
        .def_property("rotation_offset",
            [](raisim::GraphicObject &self) {  // getter
                return convert_mat_to_np(self.rotationOffset);
            }, [](raisim::GraphicObject &self, py::array_t<double> array) {  // setter
                Mat<3,3> rot = convert_np_to_mat<3,3>(array);
                self.rotationOffset = rot;
            })
        .def_readwrite("local_id", &raisim::GraphicObject::localId)
        .def_readwrite("selectable", &raisim::GraphicObject::selectable_)
        .def_readwrite("name", &raisim::GraphicObject::name)
        .def_readwrite("mesh_name", &raisim::GraphicObject::meshName)
        .def_readwrite("group", &raisim::GraphicObject::group);


    /****************/
    /* VisualObject */
    /****************/

    py::class_<raisim::VisualObject>(visualizer_module, "VisualObject", "Visual object is for visualization only")
        .def(py::init<>(), "Instantiate a visual object (by setting its orientation).")
        .def_property("pos_offset",
            [](raisim::VisualObject &self) {  // getter
                return convert_vec_to_np(self.offset);
            }, [](raisim::VisualObject &self, py::array_t<double> array) {  // setter
                Vec<3> pos = convert_np_to_vec<3>(array);
                self.offset = pos;
            })
        .def_property("scale",
            [](raisim::VisualObject &self) {  // getter
                return convert_vec_to_np(self.scale);
            }, [](raisim::VisualObject &self, py::array_t<double> array) {  // setter
                Vec<3> scale = convert_np_to_vec<3>(array);
                self.scale = scale;
            })
        .def_property("rotation_offset",
            [](raisim::VisualObject &self) {  // getter
                return convert_mat_to_np(self.rotationOffset);
            }, [](raisim::VisualObject &self, py::array_t<double> array) {  // setter
                Mat<3,3> rot = convert_np_to_mat<3,3>(array);
                self.rotationOffset = rot;
            })
        .def_readwrite("name", &raisim::VisualObject::name)
        .def_readwrite("group", &raisim::VisualObject::group);


    /****************************/
    /* SimAndGraphicsObjectPool */
    /****************************/

    py::class_<raisim::SimAndGraphicsObjectPool>(visualizer_module, "SimAndGraphicsObjectPool", "Sim and graphic object pool.")
        .def("set_world", &raisim::SimAndGraphicsObjectPool::setWorld, R"mydelimiter(
            Set the world instance.

            Args:
                world (World): world instance.
            )mydelimiter",
            py::arg("world"))

//        .def("insert", &raisim::SimAndGraphicsObjectPool::insert, R"mydelimiter(
//            Insert the given name, object and graphics.
//
//            Args:
//                name (str): object name.
//                obj (Object): object instance.
//                graphics (list[GraphicObject]): list of graphic objects associated with the object instance.
//            )mydelimiter",
//            py::arg("name"), py::arg("obj"), py::arg("graphics"))

        .def("erase", py::overload_cast<const std::string&>(&raisim::SimAndGraphicsObjectPool::erase), R"mydelimiter(
            Erase the specified object (from its name) from the world.

            Args:
                name (str): object name to be removed.
            )mydelimiter",
            py::arg("name"))

        .def("erase", py::overload_cast<Object*>(&raisim::SimAndGraphicsObjectPool::erase), R"mydelimiter(
            Erase the specified object from the world.

            Args:
                obj (Object): object instance.
            )mydelimiter",
            py::arg("name"))

        .def("sync", &raisim::SimAndGraphicsObjectPool::sync, "Synchronize: update pose of graphical objects.");


    /*************/
    /* CameraMan */
    /*************/

    py::enum_<raisim::CameraStyle>(visualizer_module, "CameraStyle")
        .value("CS_FREELOOK", raisim::CameraStyle::CS_FREELOOK)
        .value("CS_ORBIT", raisim::CameraStyle::CS_ORBIT)
        .value("CS_MANUAL", raisim::CameraStyle::CS_MANUAL);

    py::class_<raisim::CameraMan>(visualizer_module, "CameraMan", "Raisim CameraMan")
        .def("update", &raisim::CameraMan::update, "update.")
        .def("manual_stop", &raisim::CameraMan::manualStop, "Manually stops the camera when in free-look mode.")
        .def("set_fixed_yaw", &raisim::CameraMan::setFixedYaw, R"mydelimiter(
            Fix the yaw axis to be Vector3::UNIT_Y of the parent node (tabletop mode) otherwise the yaw axis can
            change freely.

            Args:
                fixed (bool): if we should fix or not the yaw axis.
            )mydelimiter",
            py::arg("fixed"))
        .def("get_top_speed", &raisim::CameraMan::getTopSpeed, R"mydelimiter(
            Get the camera top speed (only valid in the free-look style).

            Returns:
                float: camera top speed.
            )mydelimiter")
        .def("set_top_speed", &raisim::CameraMan::setTopSpeed, R"mydelimiter(
            Set the camera's top speed.

            Args:
                speed (float): camera top seed.
            )mydelimiter",
            py::arg("speed"))
        .def("get_style", &raisim::CameraMan::getStyle, R"mydelimiter(
            Get the camera style.

            Returns:
                CameraStyle: camera style between {CS_FREELOOK, CS_ORBIT, CS_MANUAL}.
            )mydelimiter")
        .def("set_style", &raisim::CameraMan::setStyle, R"mydelimiter(
            Set the camera style.

            Args:
                style (CameraStyle): camera style between {CS_FREELOOK, CS_ORBIT, CS_MANUAL}.
            )mydelimiter",
            py::arg("style"))
        .def_property("style", &raisim::CameraMan::getStyle, &raisim::CameraMan::setStyle)
        .def("set_yaw_pitch_dist", [](raisim::CameraMan &self, float yaw, float pitch, float dist, bool track) {
            self.setYawPitchDist(Ogre::Radian(yaw), Ogre::Radian(pitch), dist, track);
        }, R"mydelimiter(
            Sets the spatial offset from the target. Only applies for orbit style.

            Args:
                yaw (float): yaw angle in radians.
                pitch (float): pitch angle in radians.
                dist (float): distance from target.
                track_objects_yaw (bool): if True, tracks object yaw.
            )mydelimiter",
            py::arg("yaw"), py::arg("pitch"), py::arg("dist"), py::arg("track_objects_yaw") = false)

        .def("get_camera", &raisim::CameraMan::getCamera, R"mydelimiter(
            Get the camera scene node.

            Returns:
                Ogre.SceneNode: camera scene node.
            )mydelimiter")
        .def("set_camera", &raisim::CameraMan::setCamera, R"mydelimiter(
            Set the camera scene node; i.e. swaps the camera on our camera man for another camera.

            Args:
                camera (Ogre.SceneNode): camera scene node.
            )mydelimiter",
            py::arg("camera"))
        .def("get_target", &raisim::CameraMan::getTarget, R"mydelimiter(
            Get the target scene node that we revolve around. Only applies for orbit style.

            Returns:
                Ogre.SceneNode: target scene node.
            )mydelimiter")
        .def("set_target", &raisim::CameraMan::setCamera, R"mydelimiter(
            Set the target scene node; i.e. the target we will revolve around. Only applies for orbit style.

            Args:
                target (Ogre.SceneNode): target scene node.
            )mydelimiter",
            py::arg("target"))
        .def("set_pivot_offset", [](raisim::CameraMan &self, py::array_t<double> &offset) {
            Ogre::Vector3 vec = convert_np_to_ogre_vec3(offset);
            self.setPivotOffset(vec);
        }, R"mydelimiter(
            Set the pivot offset.

            Args:
                offset (np.array[float[3]]): pivot offset.
            )mydelimiter",
            py::arg("offset"));


    /***********/
    /* OgreVis */
    /***********/

    // py::nodedelete is because the destructor is non-public (it is private because of Singleton pattern)
    py::class_<raisim::OgreVis, std::unique_ptr<raisim::OgreVis, py::nodelete>> ogre_vis(visualizer_module, "OgreVis", "Raisim Ogre visualizer.");

    py::enum_<raisim::OgreVis::VisualizationGroup>(ogre_vis, "VisualizationGroup")
        .value("RAISIM_OBJECT_GROUP", raisim::OgreVis::VisualizationGroup::RAISIM_OBJECT_GROUP)
        .value("RAISIM_COLLISION_BODY_GROUP", raisim::OgreVis::VisualizationGroup::RAISIM_COLLISION_BODY_GROUP)
        .value("RAISIM_CONTACT_POINT_GROUP", raisim::OgreVis::VisualizationGroup::RAISIM_CONTACT_POINT_GROUP)
        .value("RAISIM_CONTACT_FORCE_GROUP", raisim::OgreVis::VisualizationGroup::RAISIM_CONTACT_FORCE_GROUP);

    ogre_vis.def(py::init([]() {
            // get reference to the Ogre visualizer
            auto vis = raisim::OgreVis::get();

            // return the visualizer
            return vis;
        }), "Initialize the visualizer.")
        .def(py::init([](raisim::World *world, uint32_t width=1280, uint32_t height=720,
                    double fps=60, int anti_aliasing=2) {
                // get reference to the Ogre visualizer
                auto vis = raisim::OgreVis::get();

                // initialize (need to be called before initApp)
                vis->setWorld(world);
                vis->setWindowSize(width, height);
                vis->setImguiSetupCallback(imguiSetupCallback);
                vis->setImguiRenderCallback(imguiRenderCallBack);
                vis->setKeyboardCallback(raisimKeyboardCallback);
                vis->setSetUpCallback(setupCallback);
                vis->setAntiAliasing(anti_aliasing);

                // starts visualizer thread (this will call `setup()`)
                vis->initApp();  // This function initializes the render system and resources. (from ApplicationContext.h)

                // set desired FPS
                vis->setDesiredFPS(fps);

                // return the visualizer
                return vis;
            }), R"mydelimiter(
            Instantiate the visualizer for the given world.

            Args:
                world (World): world instance.
                width (int): width of the window.
                height (int): height of the window.
                fps (double): the number of frames per second.
                anti_aliasing (int): anti aliasing.
            )mydelimiter",
            py::arg("world"), py::arg("width") = 1280, py::arg("height") = 720, py::arg("fps") = 60,
            py::arg("anti_aliasing") = 2)


        .def("set_default_callbacks", [](raisim::OgreVis &self) {
            auto vis = raisim::OgreVis::get();
            vis->setImguiSetupCallback(imguiSetupCallback);
            vis->setImguiRenderCallback(imguiRenderCallBack);
            vis->setKeyboardCallback(raisimKeyboardCallback);
            vis->setSetUpCallback(setupCallback);
        }, R"mydelimiter(
        This sets the default callbacks (imguiSetupCallback, imguiRenderCallBack, raisimKeyboardCallback, setupCallback).
        )mydelimiter")

        .def("init_app", [](raisim::OgreVis &self) {
            // get reference to the Ogre visualizer
            auto vis = raisim::OgreVis::get();
            // starts visualizer thread (this will call `setup()`)
            vis->initApp();
        }, R"mydelimiter(
        This calls `initApp`. Note that you have to configure it before calling it, by setting the world, window
        size, the various callbacks, and the anti-aliasing.
        )mydelimiter")

        .def("close_app", &raisim::OgreVis::closeApp, "This function closes down the application - saves the configuration then shutdowns.")

        .def("get", &raisim::OgreVis::get, R"mydelimiter(
        Return a pointer to the singleton visualizer.

        Returns:
            OgreVis: reference to this class.
        )mydelimiter")

        .def("add_resource_directory", &raisim::OgreVis::addResourceDirectory, R"mydelimiter(
            Add resource directory for materials

            Args:
                dir (str): resource directory path.
            )mydelimiter",
            py::arg("dir"))

        .def("load_material", &raisim::OgreVis::loadMaterialFile, R"mydelimiter(
            Once the directory is added (using `add_resource_dir`), load the material file in the directory.

            Args:
                filename (str): material filename.
            )mydelimiter",
            py::arg("filename"))

        .def("load_mesh", &raisim::OgreVis::loadMeshFile, R"mydelimiter(
            Loading mesh file using assimp. Ogre only reads .mesh file. If it something else is given, it creates
            .mesh file inside the directory. file can be the whole path to the meshfile, or from the memory.

            Args:
                filename (str): mesh filename.
                name (str): mesh name.
                from_memory (bool): if we should load it from memory.
            )mydelimiter",
            py::arg("filename"), py::arg("name"), py::arg("from_memory")=false)

        .def("get_resource_dir", &raisim::OgreVis::getResourceDir, R"mydelimiter(
            Get resource directory of RaisimOgreVisualizer

            Returns:
                str: resource directory path.
            )mydelimiter")

        .def("get_camera_man", &raisim::OgreVis::getCameraMan, R"mydelimiter(
            Return the camera man instance attached to the visualizer.

            Returns:
                raisim.visualizer.CameraMan: camera man instance.
            )mydelimiter")

        .def("get_light", &raisim::OgreVis::getLight, R"mydelimiter(
            Return the default light that can be adjusted.

            Returns:
                Light: Light instance.
            )mydelimiter")

        .def("get_light_node", &raisim::OgreVis::getLightNode, R"mydelimiter(
            Get LightNode for moving the main light around.

            Returns:
                SceneNode: Light node instance.
            )mydelimiter")

        .def("get_view_port", &raisim::OgreVis::getViewPort, R"mydelimiter(
            Get main Ogre::Viewport.

            Note: 'A viewport is the meeting of a camera and a rendering surface - the camera renders the scene from a
            viewpoint, and places its results into some subset of a rendering target, which may be the whole surface or
            just a part of the surface. Each viewport has a single camera as source and a single target as destination.
            A camera only has 1 viewport, but a render target may have several. A viewport also has a Z-order, i.e. if
            there is more than one viewport on a single render target and they overlap, one must obscure the other in
            some predetermined way.' [1]

            Returns:
                Viewport: the viewport.

            References:
                - [1] Ogre::Viewport class reference: https://www.ogre3d.org/docs/api/1.8/class_ogre_1_1_viewport.html
            )mydelimiter")

        .def("set_world", &raisim::OgreVis::setWorld, R"mydelimiter(
            Set the world instance to the visualizer.

            Warnings: this must be called before simulation.

            Args:
                world (World): world instance.
            )mydelimiter",
            py::arg("world"))

        .def("get_world", &raisim::OgreVis::getWorld, R"mydelimiter(
            Return the world instance attached to the visualizer.

            Returns:
                World: world instance.
            )mydelimiter")

        .def("run", &raisim::OgreVis::startRendering, "run simulation and visualization.")

        .def("start_rendering", &raisim::OgreVis::startRendering, "start rendering loop without updating simulation.")

        .def("render_one_frame", &raisim::OgreVis::renderOneFrame, "renders a single frame without updating simulation.")

        .def("set_camera_speed", &raisim::OgreVis::setCameraSpeed, R"mydelimiter(
            Set camera speed.

            Args:
                speed (float): camera speed (for free motion).
            )mydelimiter",
            py::arg("speed"))


        .def("create_graphical_object", py::overload_cast<raisim::Sphere*, const std::string&,
            const std::string&>(&raisim::OgreVis::createGraphicalObject), R"mydelimiter(
            Add a sphere in the window.

            Args:
                sphere (Sphere): Raisim sphere instance.
                name (str): name of the sphere.
                material (str): material for visualization.
            )mydelimiter",
            py::arg("sphere"), py::arg("name"), py::arg("material"))


        .def("create_graphical_object", py::overload_cast<raisim::Ground *, double, const std::string&,
            const std::string&>(&raisim::OgreVis::createGraphicalObject), R"mydelimiter(
            Add a ground in the window.

            Args:
                ground (Ground): Raisim ground instance.
                dimension (double): the plane dimension.
                name (str): name of the ground.
                material (str): material for visualization.
            )mydelimiter",
            py::arg("ground"), py::arg("dimension"), py::arg("name"), py::arg("material"))


        .def("create_graphical_object", py::overload_cast<raisim::Box*, const std::string&,
            const std::string&>(&raisim::OgreVis::createGraphicalObject), R"mydelimiter(
            Add a box in the window.

            Args:
                box (Box): Raisim box instance.
                name (str): name of the box.
                material (str): material for visualization.
            )mydelimiter",
            py::arg("box"), py::arg("name"), py::arg("material"))


        .def("create_graphical_object", py::overload_cast<raisim::Cylinder*, const std::string&,
            const std::string&>(&raisim::OgreVis::createGraphicalObject), R"mydelimiter(
            Add a cylinder in the window.

            Args:
                cylinder (Cylinder): Raisim cylinder instance.
                name (str): name of the cylinder.
                material (str): material for visualization.
            )mydelimiter",
            py::arg("cylinder"), py::arg("name"), py::arg("material"))


        .def("create_graphical_object", py::overload_cast<raisim::Wire*, const std::string&,
            const std::string&>(&raisim::OgreVis::createGraphicalObject), R"mydelimiter(
            Add a wire in the window.

            Args:
                wire (Wire): Raisim wire instance.
                name (str): name of the wire.
                material (str): material for visualization.
            )mydelimiter",
            py::arg("wire"), py::arg("name"), py::arg("material") = "default")


        .def("create_graphical_object", py::overload_cast<raisim::Capsule*, const std::string&,
            const std::string&>(&raisim::OgreVis::createGraphicalObject), R"mydelimiter(
            Add a capsule in the window.

            Args:
                capsule (Capsule): Raisim capsule instance.
                name (str): name of the capsule.
                material (str): material for visualization.
            )mydelimiter",
            py::arg("capsule"), py::arg("name"), py::arg("material"))


        .def("create_graphical_object", py::overload_cast<raisim::ArticulatedSystem*, const std::string&>
            (&raisim::OgreVis::createGraphicalObject), R"mydelimiter(
            Add an articulated system in the window.

            Args:
                articulated_system (ArticulatedSystem): Raisim articulated system instance.
                name (str): name of the articulated system.
            )mydelimiter",
            py::arg("articulated_system"), py::arg("name"))


        .def("create_graphical_object", py::overload_cast<raisim::HeightMap*, const std::string&,
            const std::string&>(&raisim::OgreVis::createGraphicalObject), R"mydelimiter(
            Add a heightmap in the window.

            Args:
                heightmap (HeightMap): Raisim heightmap instance.
                name (str): name of the heightmap.
                material (str): material for visualization.
            )mydelimiter",
            py::arg("capsule"), py::arg("name"), py::arg("material") = "default")


        .def("sync", &raisim::OgreVis::sync, "Synchronize Raisim and Ogre.")


        .def("get_paused", &raisim::OgreVis::getPaused, R"mydelimiter(
        Return if the visualizer is paused or not.

        Returns:
            bool: True if the visualizer is paused.
        )mydelimiter")
        .def_property_readonly("paused", &raisim::OgreVis::getPaused, "Return if the visualizer is paused or not.")


        .def("remove", py::overload_cast<raisim::Object*>(&raisim::OgreVis::remove), R"mydelimiter(
        Remove an object from the visualizer.

        Args:
            obj (Object): Raisim object instance to be removed.
        )mydelimiter",
        py::arg("obj"))
        .def("remove", py::overload_cast<const std::string&>(&raisim::OgreVis::remove), R"mydelimiter(
        Remove an object from the visualizer.

        Args:
            name (str): name of the object to be removed.
        )mydelimiter",
        py::arg("name"))


        .def("set_window_size", &raisim::OgreVis::setWindowSize, R"mydelimiter(
        Set the window size.

        Returns:
            width: width of the window.
            height: height of the window.
        )mydelimiter",
        py::arg("width"), py::arg("height"))


        .def("get_selected", &raisim::OgreVis::getSelected, R"mydelimiter(
        Return the current selected item.

        Returns:
            Object: Raisim object instance.
            int: index.
        )mydelimiter")


        .def("get_selected_graphical_object", &raisim::OgreVis::getSelectedGraphicalObject, R"mydelimiter(
        Return the current selected graphical object item.

        Returns:
            GraphicObject: Raisim graphic object instance.
        )mydelimiter")


        .def("select", &raisim::OgreVis::select, R"mydelimiter(
        Select the given graphic object item.

        Args:
            obj (GraphicObject): Raisim graphic object instance.
            highlight (bool): if we should highlight the graphical object in the visualizer.
        )mydelimiter",
        py::arg("obj"), py::arg("highlight") = true)


        .def("deselect", &raisim::OgreVis::deselect, "Deselect the current selected object.")


        // TODO: wrap Ogre::SceneNode
//        .def("get_raisim_object", &raisim::OgreVis::getRaisimObject, "get Raisim object.")


        // TODO: wrap Ogre::SceneNode
//        .def("get_graphic_object", &raisim::OgreVis::getGraphicObject, "get the graphic object.")


        .def("is_recording", &raisim::OgreVis::isRecording, R"mydelimiter(
        Return if the visualizer is recording or not.

        Returns:
            bool: True if the visualizer is recording.
        )mydelimiter")


        .def("start_recording_video", &raisim::OgreVis::startRecordingVideo, R"mydelimiter(
        Initiate a video recording session.

        Returns:
            filename (str): filename for the recorded video.
        )mydelimiter",
        py::arg("filename"))


        .def("stop_recording_video_and_save", &raisim::OgreVis::stopRecordingVideoAndSave,
            "Stop the recording of the video and save it in the previous given filename.")


        .def("set_desired_fps", &raisim::OgreVis::setDesiredFPS, R"mydelimiter(
        Set the desired frame per second.

        Args:
            fps (double): frame per second.
        )mydelimiter",
        py::arg("fps"))


        .def("set_anti_aliasing", &raisim::OgreVis::setDesiredFPS, R"mydelimiter(
        Set the anti-aliasing.

        Args:
            fsaa (int): frame anti-aliasing; fsaa should be set to one of (1,2,4,8).
        )mydelimiter",
        py::arg("fsaa"))


        .def("set_visibility_mask", &raisim::OgreVis::setVisibilityMask, R"mydelimiter(
        Set the visibility mask.

        Args:
            mask (unsigned long int): mask (it is a bitfield).
        )mydelimiter",
        py::arg("mask"))


        .def("get_visual_object_list", &raisim::OgreVis::getVisualObjectList, R"mydelimiter(
        Return the list of visual objects.

        Returns:
            dict[str:VisualObject]: dictionary mapping names to visual objects.
        )mydelimiter")


        .def("set_contact_visual_object_size", &raisim::OgreVis::setContactVisObjectSize, R"mydelimiter(
        Set the contact visual object size.

        Args:
            point_size (float): point size.
            force_arrow_length (float): force size corresponding to the maximum impulse.
        )mydelimiter",
        py::arg("point_size"), py::arg("force_arrow_length"))


        .def("get_real_time_factor_reference", &raisim::OgreVis::getRealTimeFactorReference, R"mydelimiter(
        Get the real time factor reference.

        Returns:
            float: real time factor reference.
        )mydelimiter")


        // TODO: improve the documentation of the below function
        .def("get_take_n_steps", &raisim::OgreVis::getTakeNSteps, R"mydelimiter(
        Get take N steps.

        Returns:
            int: num of steps.
        )mydelimiter")


        .def("set_remote_mode", &raisim::OgreVis::setRemoteMode, R"mydelimiter(
        Set the remote mode.

        Args:
            mode (bool): True if we are in a remote mode.
        )mydelimiter",
        py::arg("mode"))


        .def("remote_run", &raisim::OgreVis::remoteRun, "Run in remote mode.")


        .def("add_visual_object", [](raisim::OgreVis &self, const std::string &name, const std::string &mesh_name,
            const std::string &material, py::array_t<double> scale, bool cast_shadow = true,
            unsigned long int group = raisim::OgreVis::VisualizationGroup::RAISIM_OBJECT_GROUP |
            raisim::OgreVis::VisualizationGroup::RAISIM_COLLISION_BODY_GROUP) {
                // convert np.array to vec<3>
                raisim::Vec<3> scale_ = convert_np_to_vec<3>(scale);
                self.addVisualObject(name, mesh_name, material, scale_, cast_shadow, group);
            }, R"mydelimiter(
        Add a visual object.

        Args:
            name (str): name of the visual object.
            mesh_name (str): name of the material.
            material (str): material.
            scale (np.array[float[3]]): scale.
            cast_shadow (bool): if we should cast shadow or not.
            group (unsigned long int): group. You can select between {RAISIM_OBJECT_GROUP, RAISIM_COLLISION_BODY_GROUP,
                RAISIM_CONTACT_POINT_GROUP, RAISIM_CONTACT_FORCE_GROUP}, or any combination using bit operations.
        )mydelimiter",
        py::arg("name"), py::arg("mesh_name"), py::arg("material"), py::arg("scale"), py::arg("cast_shadow") = true,
        py::arg("group") = raisim::OgreVis::VisualizationGroup::RAISIM_OBJECT_GROUP |
            raisim::OgreVis::VisualizationGroup::RAISIM_COLLISION_BODY_GROUP)


        .def("clear_visual_object", &raisim::OgreVis::clearVisualObject, "Clear all the visual objects.")


        .def("build_height_map", &raisim::OgreVis::buildHeightMap, R"mydelimiter(
        Build the heigthmap.

        Args:
            name (str): the heightmap name.
            x_samples (int): the number of samples in x.
            x_size (float): the size in the x direction.
            x_center (float): the x center of the heightmap in the world.
            y_samples (int): the number of samples in y.
            y_size (float): the size in the y direction.
            y_center (float): the y center of the heightmap in the world.
            height (list[float]): list of desired heights.
        )mydelimiter",
        py::arg("name"), py::arg("x_samples"), py::arg("x_size"), py::arg("x_center"), py::arg("y_samples"),
        py::arg("y_size"), py::arg("y_center"), py::arg("height"));


}