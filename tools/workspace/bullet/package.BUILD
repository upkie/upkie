# -*- python -*-
#
# Copyright 2022 Stéphane Caron

package(default_visibility = ["//visibility:public"])

genrule(
    name = "check_compilation_mode",
    outs = ["compilation_mode.txt"],
    cmd  = """
    if [ $(COMPILATION_MODE) = dbg ]; then
    echo '\033[33;1mWARNING:\033[0m Building Bullet in \033[33mdbg\033[0m compilation mode, simulation will be VERY slow!';
    echo '\033[32mINFO:\033[0m Build Bullet with "-c opt" for better performance.';
    elif [ $(COMPILATION_MODE) = fastbuild ]; then
    echo '\033[33;1mWARNING:\033[0m Building Bullet in \033[33mfastbuild\033[0m compilation mode, simulation will be slow.';
    echo '\033[32mINFO:\033[0m Build Bullet with "-c opt" for better performance.';
    elif [ $(COMPILATION_MODE) = opt ]; then
    echo '\033[32mINFO:\033[0m Building Bullet in \033[32mopt\033[0m compilation mode.';
    fi;
    echo $(COMPILATION_MODE) > $@""",
)

bullet_copts = [
    "-Wno-all",
    "-Wno-deprecated-declarations",
    "-Wno-error=unused-but-set-variable",
    "-Wno-error=unused-variable",
    "-Wno-unused-result",
] + select({
    "@//:linux": [
        "-Wno-format-overflow",
        "-Wno-format-truncation",
    ],
    "@//:osx": [
        # See https://github.com/libigl/libigl/issues/751#issuecomment-383324059
        "-fno-common",
    ],
    "@//conditions:default": [],
})

bullet_defines = [
    "BT_USE_DOUBLE_PRECISION",
] + select({
    "@//:linux": [
        "DYNAMIC_LOAD_X11_FUNCTIONS=1",
        "GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1",
        "GLEW_INIT_OPENGL11_FUNCTIONS=1",
        "GLEW_STATIC",
        "HAS_SOCKLEN_T",
        "_LINUX",
    ],
    "@//:osx": [
        "B3_NO_PYTHON_FRAMEWORK",
        "GLEW_STATIC",
        "HAS_SOCKLEN_T",
        "_DARWIN",
    ],
    "@//conditions:default": [],
})

bullet_linkopts = select({
    "@//:osx": [
        "-framework Cocoa",
        "-framework OpenGL",
        "-ldl",
        # Issue errors at link time rather than at run time
        # See https://groups.google.com/g/bazel-discuss/c/YGVWGnhFEXc
        "-Wl,-undefined,error",
    ],
    "@//conditions:default": [],
})

cc_library(
    name = "src",
    srcs = glob([
        "src/Bullet3Common/**/*.cpp",
        "src/BulletCollision/**/*.cpp",
        "src/BulletDynamics/**/*.cpp",
        "src/BulletInverseDynamics/**/*.cpp",
        "src/BulletSoftBody/**/*.cpp",
        "src/LinearMath/**/*.cpp",
    ]),
    hdrs = glob([
        "src/Bullet3Collision/**/*.h",
        "src/Bullet3Common/**/*.h",
        "src/BulletCollision/**/*.h",
        "src/BulletDynamics/**/*.h",
        "src/BulletInverseDynamics/**/*.hpp",
        "src/BulletSoftBody/**/*.h",
        "src/LinearMath/**/*.h",
        "src/btBulletCollisionCommon.h",
        "src/btBulletDynamicsCommon.h",
    ]),
    defines = bullet_defines,
    includes = ["src"],
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "extras",
    srcs = glob([
        "Extras/InverseDynamics/**/*.cpp",
        "Extras/Serialize/BulletFileLoader/**/*.cpp",
        "Extras/Serialize/BulletWorldImporter/**/*.cpp",
    ]),
    hdrs = glob([
        "Extras/InverseDynamics/**/*.hpp",
        "Extras/Serialize/BulletFileLoader/**/*.h",
        "Extras/Serialize/BulletWorldImporter/**/*.h",
    ]),
    deps = [
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "common_interfaces",
    hdrs = [
        "examples/CommonInterfaces/Common2dCanvasInterface.h",
        "examples/CommonInterfaces/CommonCallbacks.h",
        "examples/CommonInterfaces/CommonCameraInterface.h",
        "examples/CommonInterfaces/CommonDeformableBodyBase.h",
        "examples/CommonInterfaces/CommonExampleInterface.h",
        "examples/CommonInterfaces/CommonFileIOInterface.h",
        "examples/CommonInterfaces/CommonGUIHelperInterface.h",
        "examples/CommonInterfaces/CommonGraphicsAppInterface.h",
        "examples/CommonInterfaces/CommonMultiBodyBase.h",
        "examples/CommonInterfaces/CommonParameterInterface.h",
        "examples/CommonInterfaces/CommonRenderInterface.h",
        "examples/CommonInterfaces/CommonRigidBodyBase.h",
        "examples/CommonInterfaces/CommonWindowInterface.h",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "buss_ik",
    srcs = glob([
        "examples/ThirdPartyLibs/BussIK/**/*.cpp",
    ]),
    hdrs = glob([
        "examples/ThirdPartyLibs/BussIK/**/*.h",
    ]),
    strip_include_prefix = "examples/ThirdPartyLibs",
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "optional_x11",
    hdrs = glob([
        "examples/ThirdPartyLibs/optionalX11/**/*.h",
    ]),
    strip_include_prefix = "examples/ThirdPartyLibs/optionalX11",
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "glad",
    srcs = glob([
        "examples/ThirdPartyLibs/glad/**/*.c",
    ]),
    hdrs = glob([
        "examples/ThirdPartyLibs/glad/**/*.h",
    ]),
    deps = [
        ":optional_x11",
    ],
    strip_include_prefix = "examples/ThirdPartyLibs/glad",
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts + [
        "-ldl",
        "-lm",
    ],
)

cc_library(
    name = "gwen",
    srcs = glob([
        "examples/ThirdPartyLibs/Gwen/**/*.cpp",
    ]),
    hdrs = glob([
        "examples/ThirdPartyLibs/Gwen/**/*.h",
    ]),
    deps = [
        ":glad",
    ],
    includes = ["examples/ThirdPartyLibs"],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "stb_image",
    srcs = glob([
        "examples/ThirdPartyLibs/stb_image/*.cpp",
    ]),
    hdrs = glob([
        "examples/ThirdPartyLibs/stb_image/*.h",
    ]),
    includes = [
        "examples/ThirdPartyLibs",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

objc_library(
    name = "mac_opengl_window",
    non_arc_srcs = [
        "examples/OpenGLWindow/MacOpenGLWindowObjC.m",
    ],
    hdrs = glob([
        "examples/OpenGLWindow/**/*.h",
    ]),
    deps = [
        ":common_interfaces",
        ":glad",
    ],
    target_compatible_with = [
        "@platforms//os:osx",
    ],
)

cc_library(
    name = "opengl_window",
    srcs = [
        "examples/OpenGLWindow/EGLOpenGLWindow.cpp",
        "examples/OpenGLWindow/fontstash.cpp",
        "examples/OpenGLWindow/GLFWOpenGLWindow.cpp",
        "examples/OpenGLWindow/GLInstancingRenderer.cpp",
        "examples/OpenGLWindow/GLPrimitiveRenderer.cpp",
        "examples/OpenGLWindow/GLRenderToTexture.cpp",
        "examples/OpenGLWindow/LoadShader.cpp",
        "examples/OpenGLWindow/opengl_fontstashcallbacks.cpp",
        "examples/OpenGLWindow/OpenSans.cpp",
        "examples/OpenGLWindow/SimpleCamera.cpp",
        "examples/OpenGLWindow/SimpleOpenGL2App.cpp",
        "examples/OpenGLWindow/SimpleOpenGL2Renderer.cpp",
        "examples/OpenGLWindow/SimpleOpenGL3App.cpp",
        "examples/OpenGLWindow/TwFonts.cpp",
    ] + select({
        "@//:linux": [
            "examples/OpenGLWindow/X11OpenGLWindow.cpp",
        ],
        "@//:osx": [
            "examples/OpenGLWindow/MacOpenGLWindow.cpp",
        ],
        # Windows is not supported:
        # "examples/OpenGLWindow/Win32OpenGLWindow.cpp",
        # "examples/OpenGLWindow/Win32Window.cpp",
        "@//conditions:default": [],
    }),
    hdrs = glob([
        "examples/OpenGLWindow/**/*.h",
    ]),
    defines = bullet_defines,
    deps = [
        ":common_interfaces",
        ":glad",
        ":src",
        ":stb_image",
    ] + select({
        "@//:linux": [],
        "@//:osx": [":mac_opengl_window"],
        "@//conditions:default": [],
    }),
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "gwen_gui_support",
    srcs = [
        "examples/ExampleBrowser/GwenGUISupport/GraphingTexture.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GwenParameterInterface.cpp",
        "examples/ExampleBrowser/GwenGUISupport/GwenTextureWindow.cpp",
        "examples/ExampleBrowser/GwenGUISupport/gwenUserInterface.cpp",
    ],
    hdrs = [
        "examples/ExampleBrowser/GwenGUISupport/GraphingTexture.h",
        "examples/ExampleBrowser/GwenGUISupport/GwenParameterInterface.h",
        "examples/ExampleBrowser/GwenGUISupport/GwenTextureWindow.h",
        "examples/ExampleBrowser/GwenGUISupport/gwenInternalData.h",
        "examples/ExampleBrowser/GwenGUISupport/gwenUserInterface.h",
    ],
    deps = [
        ":gwen",
        ":opengl_window",
        ":src",
    ],
    strip_include_prefix = "examples/ExampleBrowser",
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "tinyxml2",
    srcs = glob([
        "examples/ThirdPartyLibs/tinyxml2/*.cpp",
    ]),
    hdrs = glob([
        "examples/ThirdPartyLibs/tinyxml2/*.h",
    ]),
    includes = [
        "examples/ThirdPartyLibs",
    ],
    deps = [
        ":common_interfaces",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "wavefront",
    srcs = glob([
        "examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
    ]),
    hdrs = glob([
        "examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
    ]),
    includes = [
        "examples/ThirdPartyLibs",
    ],
    deps = [
        ":common_interfaces",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "utils",
    srcs = [
        "examples/Utils/ChromeTraceUtil.cpp",
        "examples/Utils/b3Clock.cpp",
        "examples/Utils/b3ResourcePath.cpp",
    ],
    hdrs = [
        "examples/Utils/ChromeTraceUtil.h",
        "examples/Utils/b3BulletDefaultFileIO.h",
        "examples/Utils/b3Clock.h",
        "examples/Utils/b3ERPCFMHelper.hpp",
        "examples/Utils/b3ReferenceFrameHelper.hpp",
        "examples/Utils/b3ResourcePath.h",
    ],
    deps = [
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "importers",
    srcs = [
        "examples/Importers/ImportBsp/BspConverter.cpp",
        "examples/Importers/ImportBsp/BspLoader.cpp",
        "examples/Importers/ImportBsp/ImportBspExample.cpp",
        "examples/Importers/ImportBullet/SerializeSetup.cpp",
        "examples/Importers/ImportColladaDemo/ImportColladaSetup.cpp",
        "examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
        "examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp",
        "examples/Importers/ImportMJCFDemo/ImportMJCFSetup.cpp",
        "examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
        "examples/Importers/ImportObjDemo/ImportObjExample.cpp",
        "examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp",
        "examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
        "examples/Importers/ImportSDFDemo/ImportSDFSetup.cpp",
        "examples/Importers/ImportSTLDemo/ImportSTLSetup.cpp",
        "examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
        "examples/Importers/ImportURDFDemo/ImportURDFSetup.cpp",
        "examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
        "examples/Importers/ImportURDFDemo/URDF2Bullet.cpp",
        "examples/Importers/ImportURDFDemo/UrdfParser.cpp",
        "examples/Importers/ImportURDFDemo/urdfStringSplit.cpp",
    ],
    hdrs = [
        "examples/Importers/ImportBsp/BspConverter.h",
        "examples/Importers/ImportBsp/BspLoader.h",
        "examples/Importers/ImportBsp/ImportBspExample.h",
        "examples/Importers/ImportBullet/SerializeSetup.h",
        "examples/Importers/ImportColladaDemo/ColladaGraphicsInstance.h",
        "examples/Importers/ImportColladaDemo/ImportColladaSetup.h",
        "examples/Importers/ImportColladaDemo/LoadMeshFromCollada.h",
        "examples/Importers/ImportColladaDemo/btMatrix4x4.h",
        "examples/Importers/ImportMJCFDemo/BulletMJCFImporter.h",
        "examples/Importers/ImportMJCFDemo/ImportMJCFSetup.h",
        "examples/Importers/ImportMeshUtility/b3ImportMeshUtility.h",
        "examples/Importers/ImportObjDemo/ImportObjExample.h",
        "examples/Importers/ImportObjDemo/LoadMeshFromObj.h",
        "examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.h",
        "examples/Importers/ImportSDFDemo/ImportSDFSetup.h",
        "examples/Importers/ImportSTLDemo/ImportSTLSetup.h",
        "examples/Importers/ImportSTLDemo/LoadMeshFromSTL.h",
        "examples/Importers/ImportURDFDemo/BulletUrdfImporter.h",
        "examples/Importers/ImportURDFDemo/ImportURDFSetup.h",
        "examples/Importers/ImportURDFDemo/MultiBodyCreationInterface.h",
        "examples/Importers/ImportURDFDemo/MyMultiBodyCreator.h",
        "examples/Importers/ImportURDFDemo/SDFAudioTypes.h",
        "examples/Importers/ImportURDFDemo/URDF2Bullet.h",
        "examples/Importers/ImportURDFDemo/URDFImporterInterface.h",
        "examples/Importers/ImportURDFDemo/URDFJointTypes.h",
        "examples/Importers/ImportURDFDemo/UrdfFindMeshFile.h",
        "examples/Importers/ImportURDFDemo/UrdfParser.h",
        "examples/Importers/ImportURDFDemo/UrdfRenderingInterface.h",
        "examples/Importers/ImportURDFDemo/urdfLexicalCast.h",
        "examples/Importers/ImportURDFDemo/urdfStringSplit.h",
    ],
    deps = [
        ":common_interfaces",
        ":gwen_gui_support",
        ":extras",
        ":src",
        ":tinyxml2",
        ":utils",
        ":wavefront",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "tiny_renderer",
    srcs = [
        "examples/TinyRenderer/TinyRenderer.cpp",
        "examples/TinyRenderer/geometry.cpp",
        "examples/TinyRenderer/our_gl.cpp",
        "examples/TinyRenderer/model.cpp",
        "examples/TinyRenderer/tgaimage.cpp",
    ],
    hdrs = [
        "examples/TinyRenderer/TinyRenderer.h",
        "examples/TinyRenderer/geometry.h",
        "examples/TinyRenderer/model.h",
        "examples/TinyRenderer/our_gl.h",
        "examples/TinyRenderer/tgaimage.h",
    ],
    deps = [
        ":common_interfaces",
        ":opengl_window",
        ":src",
        ":utils",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "rendering_examples",
    srcs = [
        "examples/RenderingExamples/CoordinateSystemDemo.cpp",
        "examples/RenderingExamples/DynamicTexturedCubeDemo.cpp",
        "examples/RenderingExamples/RaytracerSetup.cpp",
        "examples/RenderingExamples/RenderInstancingDemo.cpp",
        "examples/RenderingExamples/TimeSeriesCanvas.cpp",
        "examples/RenderingExamples/TimeSeriesExample.cpp",
        "examples/RenderingExamples/TimeSeriesFontData.cpp",
        "examples/RenderingExamples/TinyVRGui.cpp",
    ],
    hdrs = [
        "examples/RenderingExamples/CoordinateSystemDemo.h",
        "examples/RenderingExamples/DynamicTexturedCubeDemo.h",
        "examples/RenderingExamples/RaytracerSetup.h",
        "examples/RenderingExamples/RenderInstancingDemo.h",
        "examples/RenderingExamples/TimeSeriesCanvas.h",
        "examples/RenderingExamples/TimeSeriesExample.h",
        "examples/RenderingExamples/TimeSeriesFontData.h",
        "examples/RenderingExamples/TinyVRGui.h",
    ],
    deps = [
        ":common_interfaces",
        ":gwen_gui_support",
        ":importers",
        ":src",
        ":tiny_renderer",
        ":tinyxml2",
        ":wavefront",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "multi_threading",
    srcs = [
        "examples/MultiThreading/b3PosixThreadSupport.cpp",
        "examples/MultiThreading/b3ThreadSupportInterface.cpp",
        "examples/MultiThreading/MultiThreadingExample.cpp",
    ],
    hdrs = [
        "examples/MultiThreading/b3PosixThreadSupport.h",
        "examples/MultiThreading/b3ThreadSupportInterface.h",
        "examples/MultiThreading/MultiThreadingExample.h",
    ],
    deps = [
        ":common_interfaces",
        ":rendering_examples",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "forklift",
    srcs = [
        "examples/ForkLift/ForkLiftDemo.cpp",
    ],
    hdrs = [
        "examples/ForkLift/ForkLiftDemo.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "basic_demo",
    srcs = [
        "examples/BasicDemo/BasicExample.cpp",
    ],
    hdrs = [
        "examples/BasicDemo/BasicExample.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "gyroscopic_demo",
    srcs = [
        "examples/GyroscopicDemo/GyroscopicSetup.cpp",
    ],
    hdrs = [
        "examples/GyroscopicDemo/GyroscopicSetup.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "planar_2d",
    srcs = [
        "examples/Planar2D/Planar2D.cpp",
    ],
    hdrs = [
        "examples/Planar2D/Planar2D.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "multi_threaded_demo",
    srcs = [
        "examples/MultiThreadedDemo/CommonRigidBodyMTBase.cpp",
        "examples/MultiThreadedDemo/MultiThreadedDemo.cpp",
    ],
    hdrs = [
        "examples/MultiThreadedDemo/CommonRigidBodyMTBase.h",
        "examples/MultiThreadedDemo/MultiThreadedDemo.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "heightfield",
    srcs = [
        "examples/Heightfield/HeightfieldExample.cpp",
    ],
    hdrs = [
        "examples/Heightfield/HeightfieldExample.h",
    ],
    deps = [
        ":common_interfaces",
        ":importers",
        ":multi_threaded_demo",
        ":opengl_window",
        ":src",
        ":utils",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "benchmarks",
    srcs = [
        "examples/Benchmarks/BenchmarkDemo.cpp",
    ],
    hdrs = [
        "examples/Benchmarks/BenchmarkDemo.h",
        "examples/Benchmarks/HaltonData.h",
        "examples/Benchmarks/TaruData.h",
        "examples/Benchmarks/landscapeData.h",
    ],
    deps = [
        ":multi_threaded_demo",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "collision",
    srcs = glob([
        "examples/Collision/**/*.cpp",
    ]),
    hdrs = glob([
        "examples/Collision/**/*.h",
    ]),
    deps = [
        ":common_interfaces",
        ":rendering_examples",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "constraints",
    srcs = [
        "examples/Constraints/ConstraintDemo.cpp",
        "examples/Constraints/ConstraintPhysicsSetup.cpp",
        "examples/Constraints/Dof6Spring2Setup.cpp",
        "examples/Constraints/TestHingeTorque.cpp",
    ],
    hdrs = [
        "examples/Constraints/ConstraintDemo.h",
        "examples/Constraints/ConstraintPhysicsSetup.h",
        "examples/Constraints/Dof6Spring2Setup.h",
        "examples/Constraints/TestHingeTorque.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "multi_body",
    srcs = [
        "examples/MultiBody/InvertedPendulumPDControl.cpp",
        "examples/MultiBody/KinematicMultiBodyExample.cpp",
        "examples/MultiBody/MultiBodyConstraintFeedback.cpp",
        "examples/MultiBody/MultiBodySoftContact.cpp",
        "examples/MultiBody/MultiDofDemo.cpp",
        "examples/MultiBody/Pendulum.cpp",
        "examples/MultiBody/TestJointTorqueSetup.cpp",
    ],
    hdrs = [
        "examples/MultiBody/InvertedPendulumPDControl.h",
        "examples/MultiBody/KinematicMultiBodyExample.h",
        "examples/MultiBody/MultiBodyConstraintFeedback.h",
        "examples/MultiBody/MultiBodySoftContact.h",
        "examples/MultiBody/MultiDofDemo.h",
        "examples/MultiBody/Pendulum.h",
        "examples/MultiBody/TestJointTorqueSetup.h",
    ],
    deps = [
        ":opengl_window",
        ":src",
        ":utils",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "rigid_body",
    srcs = [
        "examples/RigidBody/KinematicRigidBodyExample.cpp",
        "examples/RigidBody/RigidBodySoftContact.cpp",
    ],
    hdrs = [
        "examples/RigidBody/KinematicRigidBodyExample.h",
        "examples/RigidBody/RigidBodySoftContact.h",
    ],
    deps = [
        ":common_interfaces",
        ":opengl_window",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "fracture_demo",
    srcs = [
        "examples/FractureDemo/FractureDemo.cpp",
        "examples/FractureDemo/btFractureBody.cpp",
        "examples/FractureDemo/btFractureDynamicsWorld.cpp",
    ],
    hdrs = [
        "examples/FractureDemo/FractureDemo.h",
        "examples/FractureDemo/btFractureBody.h",
        "examples/FractureDemo/btFractureDynamicsWorld.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "voronoi_fracture",
    srcs = [
        "examples/VoronoiFracture/VoronoiFractureDemo.cpp",
        "examples/VoronoiFracture/btConvexConvexMprAlgorithm.cpp",
    ],
    hdrs = [
        "examples/VoronoiFracture/VoronoiFractureDemo.h",
        "examples/VoronoiFracture/btConvexConvexMprAlgorithm.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "vehicles",
    srcs = [
        "examples/Vehicles/Hinge2Vehicle.cpp",
    ],
    hdrs = [
        "examples/Vehicles/Hinge2Vehicle.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "raycast",
    srcs = [
        "examples/Raycast/RaytestDemo.cpp",
    ],
    hdrs = [
        "examples/Raycast/RaytestDemo.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "dynamic_control_demo",
    srcs = [
        "examples/DynamicControlDemo/MotorDemo.cpp",
    ],
    hdrs = [
        "examples/DynamicControlDemo/MotorDemo.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "rolling_friction_demo",
    srcs = [
        "examples/RollingFrictionDemo/RollingFrictionDemo.cpp",
    ],
    hdrs = [
        "examples/RollingFrictionDemo/RollingFrictionDemo.h",
    ],
    deps = [
        ":common_interfaces",
        ":src",
        ":utils",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "meshes",
    hdrs = [
        "examples/SoftDemo/BunnyMesh.h",
        "examples/SoftDemo/bunny.inl",
        "examples/SoftDemo/cube.inl",
        "examples/SoftDemo/TorusMesh.h",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "deformable_demo",
    srcs = [
        "examples/DeformableDemo/ClothFriction.cpp",
        "examples/DeformableDemo/Collide.cpp",
        "examples/DeformableDemo/DeformableClothAnchor.cpp",
        "examples/DeformableDemo/DeformableContact.cpp",
        "examples/DeformableDemo/DeformableMultibody.cpp",
        "examples/DeformableDemo/DeformableRigid.cpp",
        "examples/DeformableDemo/DeformableSelfCollision.cpp",
        "examples/DeformableDemo/GraspDeformable.cpp",
        "examples/DeformableDemo/LargeDeformation.cpp",
        "examples/DeformableDemo/LoadDeformed.cpp",
        "examples/DeformableDemo/MultibodyClothAnchor.cpp",
        "examples/DeformableDemo/Pinch.cpp",
        "examples/DeformableDemo/PinchFriction.cpp",
        "examples/DeformableDemo/SplitImpulse.cpp",
        "examples/DeformableDemo/VolumetricDeformable.cpp",
    ],
    hdrs = [
        "examples/DeformableDemo/ClothFriction.h",
        "examples/DeformableDemo/Collide.h",
        "examples/DeformableDemo/DeformableClothAnchor.h",
        "examples/DeformableDemo/DeformableContact.h",
        "examples/DeformableDemo/DeformableMultibody.h",
        "examples/DeformableDemo/DeformableRigid.h",
        "examples/DeformableDemo/DeformableSelfCollision.h",
        "examples/DeformableDemo/GraspDeformable.h",
        "examples/DeformableDemo/LargeDeformation.h",
        "examples/DeformableDemo/LoadDeformed.h",
        "examples/DeformableDemo/MultibodyClothAnchor.h",
        "examples/DeformableDemo/Pinch.h",
        "examples/DeformableDemo/PinchFriction.h",
        "examples/DeformableDemo/SplitImpulse.h",
        "examples/DeformableDemo/VolumetricDeformable.h",
    ],
    deps = [
        ":common_interfaces",
        ":extras",
        ":importers",
        ":meshes",
        ":src",
        ":utils",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "tutorial",
    srcs = [
        "examples/Tutorial/Dof6ConstraintTutorial.cpp",
        "examples/Tutorial/Tutorial.cpp",
    ],
    hdrs = [
        "examples/Tutorial/Dof6ConstraintTutorial.h",
        "examples/Tutorial/Tutorial.h",
    ],
    deps = [
        ":common_interfaces",
        ":rendering_examples",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "inverse_dynamics",
    srcs = [
        "examples/InverseDynamics/InverseDynamicsExample.cpp",
    ],
    hdrs = [
        "examples/InverseDynamics/InverseDynamicsExample.h",
    ],
    deps = [
        ":common_interfaces",
        ":importers",
        ":rendering_examples",
        ":src",
        ":utils",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "inverse_kinematics",
    srcs = [
        "examples/InverseKinematics/InverseKinematicsExample.cpp",
    ],
    hdrs = [
        "examples/InverseKinematics/InverseKinematicsExample.h",
    ],
    deps = [
        ":buss_ik",
        ":common_interfaces",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "reduced_deformable_demo",
    srcs = glob([
        "examples/ReducedDeformableDemo/*.cpp",
    ]),
    hdrs = glob([
        "examples/ReducedDeformableDemo/*.h",
    ]),
    deps = [
        ":common_interfaces",
        ":importers",
        ":meshes",
        ":src",
        ":utils",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "extended_tutorials",
    srcs = glob([
        "examples/ExtendedTutorials/*.cpp",
    ]),
    hdrs = glob([
        "examples/ExtendedTutorials/*.h",
    ]),
    deps = [
        ":common_interfaces",
        ":importers",
        ":src",
        ":utils",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "evolution",
    srcs = [
        "examples/Evolution/NN3DWalkers.cpp",
    ],
    hdrs = [
        "examples/Evolution/NN3DWalkers.h",
        "examples/Evolution/NN3DWalkersTimeWarpBase.h",
    ],
    deps = [
        ":common_interfaces",
        ":rendering_examples",
        ":src",
        ":utils",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "robot_logging_util",
    srcs = [
        "examples/Utils/RobotLoggingUtil.cpp",
    ],
    hdrs = [
        "examples/Utils/RobotLoggingUtil.h",
    ],
    deps = [
        ":importers",
        ":src",
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "robot_simulator",
    srcs = glob([
        "examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp",
        "examples/ExampleBrowser/ExampleEntries.cpp",
        "examples/ExampleBrowser/GL_ShapeDrawer.cpp",
        "examples/ExampleBrowser/InProcessExampleBrowser.cpp",
        "examples/ExampleBrowser/OpenGLExampleBrowser.cpp",
        "examples/ExampleBrowser/OpenGLGuiHelper.cpp",
        "examples/RenderingExamples/TinyRendererSetup.cpp",
        "examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp",
        "examples/RoboticsLearning/GripperGraspExample.cpp",
        "examples/RoboticsLearning/KukaGraspExample.cpp",
        "examples/RoboticsLearning/R2D2GraspExample.cpp",  # needs PhysicsServerSharedMemory
        "examples/SharedMemory/GraphicsClientExample.cpp",
        "examples/SharedMemory/GraphicsServerExample.cpp",
        "examples/SharedMemory/IKTrajectoryHelper.cpp",
        "examples/SharedMemory/InProcessMemory.cpp",
        "examples/SharedMemory/PhysicsClient.cpp",
        "examples/SharedMemory/PhysicsClientC_API.cpp",
        "examples/SharedMemory/PhysicsClientExample.cpp",
        "examples/SharedMemory/PhysicsClientSharedMemory.cpp",
        "examples/SharedMemory/PhysicsClientSharedMemory2_C_API.cpp",
        "examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
        "examples/SharedMemory/PhysicsDirect.cpp",
        "examples/SharedMemory/PhysicsDirectC_API.cpp",
        "examples/SharedMemory/PhysicsLoopBack.cpp",
        "examples/SharedMemory/PhysicsLoopBackC_API.cpp",
        "examples/SharedMemory/PhysicsServer.cpp",
        "examples/SharedMemory/PhysicsServerCommandProcessor.cpp",
        "examples/SharedMemory/PhysicsServerExample.cpp",
        "examples/SharedMemory/PhysicsServerExampleBullet2.cpp",
        "examples/SharedMemory/PhysicsServerSharedMemory.cpp",
        "examples/SharedMemory/PosixSharedMemory.cpp",
        "examples/SharedMemory/RemoteGUIHelper.cpp",
        "examples/SharedMemory/SharedMemoryCommandProcessor.cpp",
        "examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp",
        "examples/SharedMemory/Win32SharedMemory.cpp",
        "examples/SharedMemory/b3PluginManager.cpp",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.cpp",
        "examples/SharedMemory/plugins/collisionFilterPlugin/*.cpp",
        "examples/SharedMemory/plugins/pdControlPlugin/*.cpp",
        "examples/SharedMemory/plugins/tinyRendererPlugin/*.cpp",
        "examples/SoftDemo/SoftDemo.cpp",
    ]),
    hdrs = glob([
        "examples/ExampleBrowser/CollisionShape2TriangleMesh.h",
        "examples/ExampleBrowser/EmptyExample.h",
        "examples/ExampleBrowser/ExampleBrowserInterface.h",
        "examples/ExampleBrowser/ExampleEntries.h",
        "examples/ExampleBrowser/GL_ShapeDrawer.h",
        "examples/ExampleBrowser/InProcessExampleBrowser.h",
        "examples/ExampleBrowser/OpenGLExampleBrowser.h",
        "examples/ExampleBrowser/OpenGLGuiHelper.h",
        "examples/RenderingExamples/TinyRendererSetup.h",
        "examples/RobotSimulator/b3RobotSimulatorClientAPI.h",
        "examples/RoboticsLearning/GripperGraspExample.h",
        "examples/RoboticsLearning/KukaGraspExample.h",
        "examples/RoboticsLearning/R2D2GraspExample.h",  # needs PhysicsServerSharedMemory
        "examples/SharedMemory/BodyJointInfoUtility.h",
        "examples/SharedMemory/GraphicsClientExample.h",
        "examples/SharedMemory/GraphicsServerExample.h",
        "examples/SharedMemory/GraphicsSharedMemoryBlock.h",
        "examples/SharedMemory/GraphicsSharedMemoryCommands.h",
        "examples/SharedMemory/GraphicsSharedMemoryPublic.h",
        "examples/SharedMemory/IKTrajectoryHelper.h",
        "examples/SharedMemory/InProcessMemory.h",
        "examples/SharedMemory/PhysicsClient.h",
        "examples/SharedMemory/PhysicsClientC_API.h",
        "examples/SharedMemory/PhysicsClientExample.h",
        "examples/SharedMemory/PhysicsClientSharedMemory.h",
        "examples/SharedMemory/PhysicsClientSharedMemory2_C_API.h",
        "examples/SharedMemory/PhysicsClientSharedMemory_C_API.h",
        "examples/SharedMemory/PhysicsCommandProcessorInterface.h",
        "examples/SharedMemory/PhysicsDirect.h",
        "examples/SharedMemory/PhysicsDirectC_API.h",
        "examples/SharedMemory/PhysicsLoopBack.h",
        "examples/SharedMemory/PhysicsLoopBackC_API.h",
        "examples/SharedMemory/PhysicsServer.h",
        "examples/SharedMemory/PhysicsServerCommandProcessor.h",
        "examples/SharedMemory/PhysicsServerExample.h",
        "examples/SharedMemory/PhysicsServerExampleBullet2.h",
        "examples/SharedMemory/PhysicsServerSharedMemory.h",
        "examples/SharedMemory/PosixSharedMemory.h",
        "examples/SharedMemory/RemoteGUIHelper.h",
        "examples/SharedMemory/SharedMemoryBlock.h",
        "examples/SharedMemory/SharedMemoryCommandProcessor.h",
        "examples/SharedMemory/SharedMemoryCommands.h",
        "examples/SharedMemory/SharedMemoryCommon.h",
        "examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.h",
        "examples/SharedMemory/SharedMemoryInterface.h",
        "examples/SharedMemory/SharedMemoryPublic.h",
        "examples/SharedMemory/SharedMemoryUserData.h",
        "examples/SharedMemory/Win32SharedMemory.h",
        "examples/SharedMemory/b3PluginManager.h",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_InternalData.h",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h",
        "examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h",
        "examples/SharedMemory/plugins/b3PluginAPI.h",
        "examples/SharedMemory/plugins/b3PluginCollisionInterface.h",
        "examples/SharedMemory/plugins/b3PluginContext.h",
        "examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.h",
        "examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h",
        "examples/SharedMemory/plugins/tinyRendererPlugin/*.h",
        "examples/SoftDemo/SoftDemo.h",
    ]),
    deps = [
        ":basic_demo",
        ":benchmarks",
        ":collision",
        ":common_interfaces",
        ":constraints",
        ":deformable_demo",
        ":dynamic_control_demo",
        ":evolution",
        ":extended_tutorials",
        ":extras",
        ":forklift",
        ":fracture_demo",
        ":gwen",
        ":gwen_gui_support",
        ":gyroscopic_demo",
        ":heightfield",
        ":importers",
        ":inverse_dynamics",
        ":inverse_kinematics",
        ":meshes",
        ":multi_body",
        ":multi_threaded_demo",
        ":multi_threading",
        ":planar_2d",
        ":raycast",
        ":reduced_deformable_demo",
        ":rendering_examples",
        ":rigid_body",
        ":robot_logging_util",
        ":rolling_friction_demo",
        ":src",
        ":tiny_renderer",
        ":tutorial",
        ":utils",
        ":vehicles",
        ":voronoi_fracture",
    ],
    includes = [
        "examples",                 # for user code
        "examples/ExampleBrowser",  # for SoftDemo.cpp
    ],
    defines = bullet_defines,
    copts = bullet_copts,
    linkopts = bullet_linkopts,
)

cc_library(
    name = "bullet",
    data = [
        ":check_compilation_mode",
    ],
    deps = [
        ":robot_simulator",
        ":src",
    ],
)
