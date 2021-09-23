#include "openMVG/cameras/cameras.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/exif/sensor_width_database/ParseDatabase.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/types.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress_display.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <fstream>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::image;
using namespace openMVG::sfm;

/**
 * 校验初始化的相机内参矩阵
 * @param Kmatrix   "f;0;ppx;0;f;ppy;0;0;1" 其中f\ppx\ppy是数字
 * @param focal
 * @param ppx
 * @param ppy
 * @return
 */
bool checkIntrinsicStringValidity(const std::string &Kmatrix, double &focal, double &ppx, double &ppy) {
    std::vector<std::string> vec_str;
    stl::split(Kmatrix, ';', vec_str);
    if (vec_str.size() != 9) {
        std::cerr << "\n Missing ';' character" << std::endl;
        return false;
    }
    // Check that all K matrix value are valid numbers
    for (size_t i = 0; i < vec_str.size(); ++i) {
        double readvalue = 0.0;
        std::stringstream ss;
        ss.str(vec_str[i]);
        if (!(ss >> readvalue)) {
            std::cerr << "\n Used an invalid not a number character" << std::endl;
            return false;
        }
        if (i == 0) focal = readvalue;
        if (i == 2) ppx = readvalue;
        if (i == 5) ppy = readvalue;
    }
    return true;
}


int main()
{
    // 图像数据输入
    std::string sImageDir = "../datainput/imgs";
    // 相机型号数据库
    std::string sfileDatabase = "../datainput/sensor/sensor_width_camera_database.txt";
    // 图像json信息输出路径
    std::string sOutputDir = "../dataoutput";
    // 相机内参
    std::string sKmatrix;
    // 相机模型, 径向畸变k1,k2,k3
    int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;
    // 相同型号相机组合在一起（如果需要），能获得更快更稳定的BA
    bool b_Group_camera_model = true;
    // 焦距(单位像素)
    double focal_pixels = -1.0;
    // 对于每张图像
    double width = -1, height = -1,  ppx = -1, ppy = -1;
    // 焦距(单位毫米)
    double focal = -1;

    const EINTRINSIC e_User_camera_model = EINTRINSIC(i_User_camera_model);

    // 输入输出校验
    if (!stlplus::folder_exists(sImageDir)) {
        std::cerr << "\nThe input directory doesn't exist" << std::endl;
        return EXIT_FAILURE;
    }
    if (sOutputDir.empty()) {
        std::cerr << "\nInvalid output directory" << std::endl;
        return EXIT_FAILURE;
    }
    if(!stlplus::folder_exists(sOutputDir)){
        if (!stlplus::folder_create(sOutputDir)) {
            std::cerr << "\nCannot create output directory" << std::endl;
            return EXIT_FAILURE;
        }
    }
    if (sKmatrix.size() > 0 && !checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy)) {
        std::cerr << "\nInvalid K matrix input" << std::endl;
        return EXIT_FAILURE;
    }
    // 不能同时赋值相机内参和焦距(因为对应焦距是计算出来的)
    if (sKmatrix.size() > 0 && focal_pixels != -1.0) {
        std::cerr << "\nCannot combine -f and -k options" << std::endl;
        return EXIT_FAILURE;
    }

    // 获取图像exif信息存储到vec_database : Datasheet（相机型号，ccd大小mm）
    std::vector<Datasheet> vec_database;
    if (!sfileDatabase.empty()) {
        if (!parseDatabase(sfileDatabase, vec_database)) {
            std::cerr << "\nInvalid input database: " << sfileDatabase
                      << ", please specify a valid file." << std::endl;
            return EXIT_FAILURE;
        }
    }
    std::vector<std::string> vec_image = stlplus::folder_files(sImageDir);
    std::sort(vec_image.begin(), vec_image.end());

    // SfM_Data 包含图像物理信息，内参与外参。
    SfM_Data sfm_data;
    sfm_data.s_root_path = sImageDir; //
    Views &views = sfm_data.views;
    Intrinsics &intrinsics = sfm_data.intrinsics;

    // 进度条
    C_Progress_display my_progress_bar(vec_image.size(), std::cout, "\n- Image listing -\n");
    std::ostringstream error_report_stream;
    for (std::vector<std::string>::const_iterator iter_image = vec_image.begin(); iter_image != vec_image.end(); ++iter_image, ++my_progress_bar){
        width = height = ppx = ppy = focal = -1.0;
        const std::string sImageFilename = stlplus::create_filespec(sImageDir, *iter_image);
        const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);
        // 校验图片文件格式
        if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown) {
            error_report_stream << sImFilenamePart << ": Unkown image file format." << "\n";
            continue; // image cannot be opened
        }

        ImageHeader imgHeader;
        if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader)){
            continue; // image cannot be read
        }

        width = imgHeader.width;
        height = imgHeader.height;
        ppx = width / 2.0;
        ppy = height / 2.0;

        // 手动提供焦距mm
        if (sKmatrix.size() > 0) {// Known user calibration K matrix
            if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy)){
                focal = -1.0;
            }
        } else if(focal_pixels != -1){// User provided focal length value
            focal = focal_pixels;
        }

        // 没有提供焦距或者提供错误的焦距，就需要计算
        if (focal == -1) {
            std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
            exifReader->open(sImageFilename);
            // 图像包含元信息
            const bool bHaveValidExifMetadata = exifReader->doesHaveExifInfo() && !exifReader->getModel().empty();
            if (bHaveValidExifMetadata) {
                const std::string sCamModel = exifReader->getModel();
                if (exifReader->getFocal() == 0.0f) { // 校验focal
                    error_report_stream << stlplus::basename_part(sImageFilename) << ": Focal length is missing." << "\n";
                    focal = -1.0;
                } else {
                    Datasheet datasheet;
                    if (getInfo(sCamModel, vec_database, datasheet)) {
                        // 找到相机型号，就能计算近似焦距(单位像素)
                        const double ccdw = datasheet.sensorSize_;
                        /** 相机焦距计算（单位像素）
                        focal = max(w, h) * F / S
                        F：相机焦距长度(focal length，单位毫米mm)
                        w,h: 图片的宽高
                        S: ccd传感器孔径（sensor size，单位mm）*/
                        focal = std::max(width, height) * exifReader->getFocal() / ccdw;
                    } else {
                        error_report_stream << stlplus::basename_part(sImageFilename)
                                << "\" model \"" << sCamModel << "\" doesn't exist in the database" << "\n"
                                << "Please consider add your camera model and sensor width in the database." << "\n";
                    }
                }
            }
        }

        // 构建与视图的内参
        std::shared_ptr<IntrinsicBase> intrinsic;

        if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0) {
            // Create the desired camera type
            switch (e_User_camera_model) {
                case PINHOLE_CAMERA:
                    intrinsic = std::make_shared<Pinhole_Intrinsic>(width, height, focal, ppx, ppy);
                    break;
                case PINHOLE_CAMERA_RADIAL1:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K1>
                            (width, height, focal, ppx, ppy, 0.0); // setup no distortion as initial guess
                    break;
                case PINHOLE_CAMERA_RADIAL3:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
                            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
                    break;
                case PINHOLE_CAMERA_BROWN:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Brown_T2>
                            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0,
                             0.0); // setup no distortion as initial guess
                    break;
                case PINHOLE_CAMERA_FISHEYE:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Fisheye>
                            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0,
                             0.0); // setup no distortion as initial guess
                    break;
                case CAMERA_SPHERICAL:
                    intrinsic = std::make_shared<Intrinsic_Spherical>
                            (width, height);
                    break;
                default:
                    std::cerr << "Error: unknown camera model: " << (int) e_User_camera_model << std::endl;
                    return EXIT_FAILURE;
            }
        }
        // 构建与图像相关的视图信息
        View v(*iter_image, views.size(), views.size(), views.size(), width, height);

        // Add intrinsic related to the image (if any)
        if (intrinsic == nullptr) {
            //Since the view have invalid intrinsic data
            // (export the view, with an invalid intrinsic field value)
            v.id_intrinsic = UndefinedIndexT;
        } else {
            // Add the defined intrinsic to the sfm_container
            intrinsics[v.id_intrinsic] = intrinsic;
        }

        // Add the view to the sfm_container
        views[v.id_view] = std::make_shared<View>(v);
    }

    // 打印错误信息
    if (!error_report_stream.str().empty()) {
        std::cerr << "\nWarning & Error messages:" << std::endl
                  << error_report_stream.str() << std::endl;
    }

    // 相同型号相机组合在一起（如果需要），能获得更快更稳定的BA
    if (b_Group_camera_model) {
        GroupSharedIntrinsics(sfm_data);
    }

    // 存储视图和内参到json文件
    if (!Save(sfm_data, stlplus::create_filespec(sOutputDir, "sfm_data.json").c_str(),ESfM_Data(VIEWS | INTRINSICS))) {
        return EXIT_FAILURE;
    }

    // 打印report
    std::cout << std::endl
              << "SfMInit_ImageListing report:\n"
              << "listed #File(s): " << vec_image.size() << "\n"
              << "usable #File(s) listed in sfm_data: " << sfm_data.GetViews().size() << "\n"
              << "usable #Intrinsic(s) listed in sfm_data: " << sfm_data.GetIntrinsics().size() << std::endl;

    return EXIT_SUCCESS;

}