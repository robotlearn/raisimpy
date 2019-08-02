/**
 * Python wrappers for Ogre using pybind11.
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

// include Ogre related headers
#include "Ogre.h"                      // Basic Ogre classes
#include "OgreInput.h"                 // Keyboard event
#include "OgreApplicationContext.h"    // initApp, closeApp, setWindowSize, etc


// include headers that allows to convert between raisim::Vec, raisim::Mat, raisim::VecDyn, raisim::Mat to np.arrays.
#include "converter.hpp"

namespace py = pybind11;


void init_ogre(py::module &m) {

    // create submodules
    py::module ogre_module = m.def_submodule("ogre", "Ogre submodule");

    /********/
    /* Ogre */
    /********/

    /*******************/
    /* ShadowTechnique */
    /*******************/
    py::enum_<Ogre::ShadowTechnique>(ogre_module, "ShadowTechnique")
        .value("SHADOWTYPE_NONE", Ogre::ShadowTechnique::SHADOWTYPE_NONE, "No shadows.")
        .value("SHADOWTYPE_TEXTURE_MODULATIVE", Ogre::ShadowTechnique::SHADOWTYPE_TEXTURE_MODULATIVE,
            "Texture-based shadow technique which involves a monochrome render-to-texture of the shadow caster and "
            "a projection of that texture onto the shadow receivers as a modulative pass.")
        .value("SHADOWTYPE_TEXTURE_ADDITIVE", Ogre::ShadowTechnique::SHADOWTYPE_TEXTURE_ADDITIVE,
            "Texture-based shadow technique which involves a render-to-texture of the shadow caster and a projection "
            "of that texture onto the shadow receivers, built up per light as additive passes.");
    // TODO: add the other ShadowTechnique


    /**************************/
    /* Keysym & KeyboardEvent */
    /**************************/
    py::class_<OgreBites::Keysym>(ogre_module, "Keysym")
        .def_readwrite("sym", &OgreBites::Keysym::sym)
        .def_readwrite("mod", &OgreBites::Keysym::mod);

    py::class_<OgreBites::KeyboardEvent>(ogre_module, "KeyboardEvent")
        .def_readwrite("type", &OgreBites::KeyboardEvent::type)
        .def_readwrite("keysym", &OgreBites::KeyboardEvent::keysym)
        .def_readwrite("repeat", &OgreBites::KeyboardEvent::repeat);


    /***************/
    /* PixelFormat */
    /***************/
    py::enum_<Ogre::PixelFormat>(ogre_module, "PixelFormat")
        .value("PF_UNKNOWN", Ogre::PixelFormat::PF_UNKNOWN, "Unknown pixel format.")
        .value("PF_X8R8G8B8", Ogre::PixelFormat::PF_X8R8G8B8, "32-bit pixel format, 8 bits for red, 8 bits for green, 8 bits for blue.");
    // TODO: add the other PixelFormat


    /****************/
    /* SceneManager */
    /****************/
    // https://www.ogre3d.org/docs/api/1.9/class_ogre_1_1_scene_manager.html
    py::class_<Ogre::SceneManager>(ogre_module, "SceneManager",
        "Ogre SceneManager: Manages the organisation and rendering of a 'scene', i.e. a collection of objects and "
        "potentially world geometry.")
        .def("set_shadow_technique", &Ogre::SceneManager::setShadowTechnique, R"mydelimiter(
            Set the shadow technique.

            Args:
                technique (Ogre.ShadowTechnique): shadow technique.
            )mydelimiter",
            py::arg("technique"))
        .def("set_shadow_texture_settings", &Ogre::SceneManager::setShadowTextureSettings, R"mydelimiter(
            Set the size and count of textures used in texture-based shadows.

            Args:
                size (uint16): size.
                count (uint16): count.
                format (Ogre.PixelFormat): pixel format (by default, RGB).
                fsaa (uint16): anti-aliasing.
                depth_buffer_pool_id (uint16): depth buffer pool id.
            )mydelimiter",
            py::arg("size"), py::arg("count"), py::arg("format") = Ogre::PixelFormat::PF_X8R8G8B8, py::arg("fsaa")=0, py::arg("depth_buffer_pool_id")=1)
        .def("set_shadow_far_distance", &Ogre::SceneManager::setShadowFarDistance, R"mydelimiter(
            Set the default maximum distance away from the camera that shadows will be visible. You have to call
            this function before you create lights or the default distance of zero will be used.

            Args:
                distance (float): distance.
            )mydelimiter",
            py::arg("distance"))
        ;


    /********/
    /* Node */
    /********/
    py::class_<Ogre::Node> node(ogre_module, "Node", "Ogre Node: Class representing a general-purpose node an articulated scene graph.");

    py::enum_<Ogre::Node::TransformSpace>(node, "TransformSpace")
        .value("TS_LOCAL", Ogre::Node::TransformSpace::TS_LOCAL, "Transform is relative to the local space")
        .value("TS_PARENT", Ogre::Node::TransformSpace::TS_PARENT, "Transform is relative to the space of the parent node")
        .value("TS_WORLD", Ogre::Node::TransformSpace::TS_WORLD, "Transform is relative to world space");

    node.def("get_position", [](Ogre::Node &self) {
            Ogre::Vector3 vec = self.getPosition();
            return convert_ogre_vec3_to_np(vec);
        }, R"mydelimiter(
            Get the node position.

            Returns:
                np.array[float[3]]: position vector.
            )mydelimiter")
        .def("set_position", py::overload_cast<float, float, float>(&Ogre::Node::setPosition), R"mydelimiter(
            Set the node position.

            Args:
                x (float): x position.
                y (float): y position.
                z (float): z position.
            )mydelimiter",
            py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_position", [](Ogre::Node &self, py::array_t<double> &pos) {
            Ogre::Vector3 vec = convert_np_to_ogre_vec3(pos);
            self.setPosition(vec);
        }, R"mydelimiter(
            Set the node position.

            Args:
                pos (np.array[float[3]]): position vector.
            )mydelimiter",
            py::arg("pos"))

        .def("get_orientation", [](Ogre::Node &self) {
            Ogre::Quaternion quat = self.getOrientation();
            return convert_ogre_quat_to_np(quat);
        }, R"mydelimiter(
            Get the node orientation (expressed as a quaternion).

            Returns:
                np.array[float[4]]: quaternion [w,x,y,z].
            )mydelimiter")
        .def("set_orientation", [](Ogre::Node &self, py::array_t<double> &quat) {
            Ogre::Quaternion q = convert_np_to_ogre_quat(quat);
            self.setOrientation(q);
        }, R"mydelimiter(
            Set the node orientation (expressed as a quaternion).

            Args:
                quat (np.array[float[4]]): quaternion [w,x,y,z].
            )mydelimiter",
            py::arg("quat"))

        .def("get_scale", [](Ogre::Node &self) {
            Ogre::Vector3 vec = self.getScale();
            return convert_ogre_vec3_to_np(vec);
        }, R"mydelimiter(
            Get the node scale.

            Returns:
                np.array[float[3]]: scale vector.
            )mydelimiter")
        .def("set_scale", [](Ogre::Node &self, py::array_t<double> &scale) {
            Ogre::Vector3 vec = convert_np_to_ogre_vec3(scale);
            self.setScale(vec);
        }, R"mydelimiter(
            Set the node scale.

            Args:
                scale (np.array[float[3]]): scale vector.
            )mydelimiter",
            py::arg("scale"))

        .def("pitch", [](Ogre::Node &self, float angle, Ogre::Node::TransformSpace relative_to) {
            self.pitch(Ogre::Radian(angle), relative_to);
        }, R"mydelimiter(
            Rotate the node around the X-axis.

            Args:
                angle (float): radian angle.
                relative_to (TransformSpace): transform space relative to.
            )mydelimiter",
            py::arg("angle"), py::arg("relative_to") = Ogre::Node::TransformSpace::TS_LOCAL);



    /*************/
    /* SceneNode */
    /*************/
    py::class_<Ogre::SceneNode, Ogre::Node>(ogre_module, "SceneNode",
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
            py::arg("visible"), py::arg("cascade")=true);
        // add other methods if necessary from:
        // - https://github.com/OGRECave/ogre/blob/master/OgreMain/include/OgreSceneNode.h
        // - https://github.com/OGRECave/ogre/blob/master/OgreMain/include/OgreNode.h



    /*********/
    /* Light */
    /*********/
    py::class_<Ogre::Light> light(ogre_module, "Light",
        "Ogre Light: Lights are added to the scene like any other object. They contain various "
        "parameters like type, position, attenuation (how light intensity fades with distance), colour etc.");

    py::enum_<Ogre::Light::LightTypes>(light, "LightTypes")
        .value("LT_POINT", Ogre::Light::LightTypes::LT_POINT, "Point light sources give off light equally in all directions, so require only position not direction")
        .value("LT_DIRECTIONAL", Ogre::Light::LightTypes::LT_DIRECTIONAL, "Directional lights simulate parallel light beams from a distant source, hence have direction but no position")
        .value("LT_SPOTLIGHT", Ogre::Light::LightTypes::LT_SPOTLIGHT, "Spotlights simulate a cone of light from a source so require position and direction, plus extra values for falloff");

    light.def(py::init<>(), "Default light constructor")
        .def(py::init<const std::string &>(), "Normal constructor. Should not be called directly, but rather the SceneManager::createLight method should be used.")
        .def("set_diffuse_color", py::overload_cast<float, float, float>(&Ogre::Light::setDiffuseColour), R"mydelimiter(
            Sets the colour of the diffuse light given off by this source.

            Args:
                red (float): red channel value.
                green (float): green channel value.
                blue (float): blue channel value.
            )mydelimiter",
            py::arg("red"), py::arg("green"), py::arg("blue"))
        .def("set_cast_shadows", &Ogre::Light::setCastShadows, R"mydelimiter(
            Sets whether or not this object will cast shadows.

            Args:
                enabled (bool): enable casting shadows or not.
            )mydelimiter",
            py::arg("enabled"))
        ;
        // add other methods if necessary from https://github.com/OGRECave/ogre/blob/master/OgreMain/include/OgreLight.h


    /************/
    /* Viewport */
    /************/
    py::class_<Ogre::Viewport>(ogre_module, "Viewport", "Ogre viewport: An abstraction of a viewport, i.e. a rendering region on a render target.");

    // TODO: you can also check the Ogre::Camera class from https://ogrecave.github.io/ogre/api/1.10/class_ogre_1_1_camera.html
    // You can get the projection and view matrix for instance.


}