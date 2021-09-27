// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2016 cDc <cdc.seacave@gmail.com>, Pierre MOULON

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/cameras/Camera_undistort_image.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#define _USE_EIGEN
#include "InterfaceMVS.h"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/progress/progress_display.hpp"

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::image;
using namespace openMVG::sfm;

#include <atomic>
#include <cstdlib>
#include <string>

#include <omp.h>

bool exportToOpenMVS(const SfM_Data & sfm_data,const std::string & sOutFile,const std::string & sOutDir,const int iNumThreads = 0)
{
    // 创建无畸变的图像文件夹
    if (!stlplus::is_folder(sOutDir)){
        stlplus::folder_create(sOutDir);
        if (!stlplus::is_folder(sOutDir)){
            std::cerr << "Cannot access to one of the desired output directory" << std::endl;
            return false;
        }
    }

    // 初始化
    MVS::Interface scene;
    size_t nPoses(0);
    const uint32_t nViews((uint32_t)sfm_data.GetViews().size());

    // 进度条
    C_Progress_display my_progress_bar(nViews, std::cout, "\n- PROCESS VIEWS -\n");
    // 创建连续索引
    std::map<openMVG::IndexT, uint32_t> map_intrinsic, map_view;

    // platforms :  platform : cameras
    for (const auto& intrinsic: sfm_data.GetIntrinsics()){
        // 分为针孔相机模型和广角相机模型
        // https://blog.csdn.net/qq_28087491/article/details/107965151
        if (isPinhole(intrinsic.second->getType())){
            const Pinhole_Intrinsic * cam = dynamic_cast<const Pinhole_Intrinsic*>(intrinsic.second.get());
            if (map_intrinsic.count(intrinsic.first) == 0)
                map_intrinsic.insert(std::make_pair(intrinsic.first, scene.platforms.size()));
            MVS::Interface::Platform platform;
            // add the camera
            MVS::Interface::Platform::Camera camera;
            camera.width = cam->w();
            camera.height = cam->h();
            camera.K = cam->K();
            // sub-pose
            camera.R = Mat3::Identity();
            camera.C = Vec3::Zero();
            platform.cameras.push_back(camera);
            scene.platforms.push_back(platform);
        }
    }
    // 定义 images 和 poses
    scene.images.reserve(nViews);
    for (const auto& view : sfm_data.GetViews()) {
        ++my_progress_bar;

        const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view.second->s_Img_path);
        if (!stlplus::is_file(srcImage)){
            std::cerr << "Cannot read the corresponding image: " << srcImage << std::endl;
            return false;
        }

        if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get())){
            map_view[view.first] = scene.images.size();

            MVS::Interface::Image image;
            image.name = stlplus::create_filespec(sOutDir, view.second->s_Img_path);
            image.platformID = map_intrinsic.at(view.second->id_intrinsic);
            MVS::Interface::Platform& platform = scene.platforms[image.platformID];
            image.cameraID = 0;

            MVS::Interface::Platform::Pose pose;
            image.poseID = platform.poses.size();
            const openMVG::geometry::Pose3 poseMVG(sfm_data.GetPoseOrDie(view.second.get()));
            pose.R = poseMVG.rotation();
            pose.C = poseMVG.center();
            platform.poses.push_back(pose);
            ++nPoses;
            scene.images.emplace_back(image);
        }  else {
            std::cout << "Cannot read the corresponding pose or intrinsic of view " << view.first << std::endl;
        }
    }

    // 导出无畸变图像
    C_Progress_display my_progress_bar_images(sfm_data.views.size(),std::cout, "\n- UNDISTORT IMAGES -\n" );
    std::atomic<bool> bOk(true); // Use a boolean to track the status of the loop process
#ifdef OPENMVG_USE_OPENMP
    const unsigned int nb_max_thread = (iNumThreads > 0)? iNumThreads : omp_get_max_threads();

  #pragma omp parallel for schedule(dynamic) num_threads(nb_max_thread)
#endif
    for (int i = 0; i < static_cast<int>(sfm_data.views.size()); ++i) {
        ++my_progress_bar_images;

        if (!bOk)  continue;

        Views::const_iterator iterViews = sfm_data.views.begin();
        std::advance(iterViews, i);
        const View * view = iterViews->second.get();

        // 获取图像路径
        const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
        const std::string imageName = stlplus::create_filespec(sOutDir, view->s_Img_path);

        if (sfm_data.IsPoseAndIntrinsicDefined(view)) {
            // export undistorted images
            const openMVG::cameras::IntrinsicBase * cam = sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
            if (cam->have_disto()) {
                // undistort image and save it
                Image<openMVG::image::RGBColor> imageRGB, imageRGB_ud;
                Image<uint8_t> image_gray, image_gray_ud;
                try {
                    if (ReadImage(srcImage.c_str(), &imageRGB)) {
                        UndistortImage(imageRGB, cam, imageRGB_ud, BLACK);
                        bOk = WriteImage(imageName.c_str(), imageRGB_ud);
                    } else if (ReadImage(srcImage.c_str(), &image_gray)){ // If RGBColor reading fails, try to read as gray image
                        UndistortImage(image_gray, cam, image_gray_ud, BLACK);
                        const bool bRes = WriteImage(imageName.c_str(), image_gray_ud);
                        bOk = bOk & bRes;
                    } else {
                        bOk = false;
                    }
                } catch (const std::bad_alloc& e) {
                    bOk = false;
                }
            } else {
                // just copy image
                stlplus::file_copy(srcImage, imageName);
            }
        } else {
            // just copy the image
            stlplus::file_copy(srcImage, imageName);
        }
    }

    if (!bOk){
        std::cerr << "Catched a memory error in the image conversion."
                  << " Please consider to use less threads ([-n|--numThreads])." << std::endl;
        return false;
    }

    // define structure
    scene.vertices.reserve(sfm_data.GetLandmarks().size());
    for (const auto& vertex: sfm_data.GetLandmarks()) {
        const Landmark & landmark = vertex.second;
        MVS::Interface::Vertex vert;
        MVS::Interface::Vertex::ViewArr& views = vert.views;
        for (const auto& observation: landmark.obs){
            const auto it(map_view.find(observation.first));
            if (it != map_view.end()) {
                MVS::Interface::Vertex::View view;
                view.imageID = it->second;
                view.confidence = 0;
                views.push_back(view);
            }
        }
        if (views.size() < 2) continue;
        std::sort(views.begin(), views.end(),
                  [] (const MVS::Interface::Vertex::View& view0, const MVS::Interface::Vertex::View& view1) {
                    return view0.imageID < view1.imageID;
                    }
        );
        vert.X = landmark.X.cast<float>();
        scene.vertices.push_back(vert);
    }

    // 写入openMVS数据
    if (!MVS::ARCHIVE::SerializeSave(scene, sOutFile))
        return false;

    std::cout << "Scene saved to OpenMVS interface format:\n"
              << " #platforms: " << scene.platforms.size() << std::endl;
    for (int i = 0; i < scene.platforms.size(); ++i) {
        std::cout << "  platform ( " << i << " ) #cameras: " << scene.platforms[i].cameras.size() << std::endl;
    }
    std::cout << "  " << scene.images.size() << " images (" << nPoses << " calibrated)\n"
              << "  " << scene.vertices.size() << " Landmarks\n";
    return true;
}


int main()
{
    // MVG产生的sfm_data.bin文件
    std::string sSfM_Data_Filename = "../dataoutput/sfm_out/sfm_data.bin";
    // 输出文件
    std::string sOutFile = "scene.mvs";
    std::string sOutDir = "../dataoutput/undistorted_images/";
    int iNumThreads = 2;

    if (stlplus::extension_part(sOutFile) != "mvs") {
        std::cerr << std::endl
                  << "Invalid output file extension: " << sOutFile << std::endl
                  << "You must use a filename with a .mvs extension." << std::endl;
        return EXIT_FAILURE;
    }

    // 读取输入sfm_data.bin的场景数据
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(ALL))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
        return EXIT_FAILURE;
    }

    // 导出OpenMVS数据结构
    if (!exportToOpenMVS(sfm_data, sOutFile, sOutDir, iNumThreads))
    {
        std::cerr << std::endl << "The output openMVS scene file cannot be written" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}