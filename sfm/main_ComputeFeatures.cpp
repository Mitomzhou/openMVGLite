#include <cereal/archives/json.hpp>
#include <cereal/details/helpers.hpp>

#include "openMVG/features/akaze/image_describer_akaze_io.hpp"

#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer_io.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/regions_factory_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/system/timer.hpp"

#include "third_party/progress/progress_display.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <atomic>
#include <cstdlib>
#include <fstream>
#include <string>

#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif

using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::features;
using namespace openMVG::sfm;
using namespace std;

features::EDESCRIBER_PRESET stringToEnum(const std::string & sPreset)
{
    features::EDESCRIBER_PRESET preset;
    if (sPreset == "NORMAL")
        preset = features::NORMAL_PRESET;
    else
    if (sPreset == "HIGH")
        preset = features::HIGH_PRESET;
    else
    if (sPreset == "ULTRA")
        preset = features::ULTRA_PRESET;
    else
        preset = features::EDESCRIBER_PRESET(-1);
    return preset;
}


int main() {
    // main_SfMInit_ImageListing生成的json文件的路径
    std::string sSfM_Data_Filename = "../dataoutput/sfm_data.json";
    // 特征文件和秒睡文件输出到matches文件夹
    std::string sOutDir = "../dataoutput/matches";
    // AKAZE描述子使用，是否计算方向
    bool bUpRight = false;
    // 使用的描述子:"nonFree/sift/SIFT_describer_io.hpp" 引用出问题
    std::string sImage_Describer_Method = "SIFT_ANATOMY";
    //
    bool bForce = true;
    // 描述子质量:normal high ultra
    std::string sFeaturePreset = "";

#ifdef OPENMVG_USE_OPENMP
    int iNumThreads = 6;
#endif


    // 校验输入输出
    if (sOutDir.empty()) {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return EXIT_FAILURE;
    }
    if (!stlplus::folder_exists(sOutDir)) {
        if (!stlplus::folder_create(sOutDir)) {
            std::cerr << "Cannot create output directory" << std::endl;
            return EXIT_FAILURE;
        }
    }

    // 加载输入视图
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS))) {
        std::cerr << std::endl << "The input file \"" << sSfM_Data_Filename << "\" cannot be read" << std::endl;
        return EXIT_FAILURE;
    }

    using namespace openMVG::features;
    std::unique_ptr<Image_describer> image_describer;
    // 创建描述json
    const std::string sImage_describer = stlplus::create_filespec(sOutDir, "image_describer", "json");

    // 特征描述子方法
    if (sImage_Describer_Method == "SIFT") {
        // image_describer.reset(new SIFT_Image_describer(SIFT_Image_describer::Params(), !bUpRight));
    } else if (sImage_Describer_Method == "SIFT_ANATOMY") {
        image_describer.reset(new SIFT_Anatomy_Image_describer(SIFT_Anatomy_Image_describer::Params()));
    } else if (sImage_Describer_Method == "AKAZE_FLOAT") {
        image_describer = AKAZE_Image_describer::create(AKAZE_Image_describer::Params(AKAZE::Params(), AKAZE_MSURF),
                                                        !bUpRight);
    } else if (sImage_Describer_Method == "AKAZE_MLDB") {
        image_describer = AKAZE_Image_describer::create(AKAZE_Image_describer::Params(AKAZE::Params(), AKAZE_MLDB),
                                                        !bUpRight);
    }

    if (!image_describer) {
        std::cerr << "Cannot create the designed Image_describer:"
                  << sImage_Describer_Method << "." << std::endl;
        return EXIT_FAILURE;
    } else {
        if (!sFeaturePreset.empty()) {
            if (!image_describer->Set_configuration_preset(stringToEnum(sFeaturePreset))) {
                std::cerr << "Preset configuration failed." << std::endl;
                return EXIT_FAILURE;
            }
        }
    }
    //设置好描述子类型与描述子质量后
    //导出用于：动态未来区域计算和/或加载的所用图像描述符和区域类型
    {
        std::ofstream stream(sImage_describer.c_str());
        if (!stream.is_open()) {
            return EXIT_FAILURE;
        }
        cereal::JSONOutputArchive archive(stream);
        archive(cereal::make_nvp("image_describer", image_describer));
        auto regionsType = image_describer->Allocate();
        archive(cereal::make_nvp("regions_type", regionsType));
    }

    {
        system::Timer timer;
        Image<unsigned char> imageGray;

        // 特征提取进度条
        C_Progress_display my_progress_bar(sfm_data.GetViews().size(), std::cout, "\n- EXTRACT FEATURES -\n");
        // Use a boolean to track if we must stop feature extraction
        std::atomic<bool> preemptive_exit(false);

#ifdef OPENMVG_USE_OPENMP
        const unsigned int nb_max_thread = omp_get_max_threads();
        if (iNumThreads > 0) {
            omp_set_num_threads(iNumThreads);
        } else {
            omp_set_num_threads(nb_max_thread);
        }
#pragma omp parallel for schedule(dynamic) if (iNumThreads > 0) private(imageGray)
#endif
        // 开始计算特征
        for (int i = 0; i < static_cast<int>(sfm_data.views.size()); ++i) {
            Views::const_iterator iterViews = sfm_data.views.begin();
            std::advance(iterViews, i);
            const View *view = iterViews->second.get();

            // 创建两个文件 .feat .desc
            const std::string sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
            const std::string sFeat = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "feat");
            const std::string sDesc = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "desc");

            // 不存在就计算
            if (!preemptive_exit && (bForce || !stlplus::file_exists(sFeat) || !stlplus::file_exists(sDesc))) {
                if (!ReadImage(sView_filename.c_str(), &imageGray)) {
                    continue;
                }

                // 查看是否有遮挡特征mask，默认是没有mask
                Image<unsigned char> *mask = nullptr;

                const std::string mask_filename_local = stlplus::create_filespec(sfm_data.s_root_path,
                                                                                 stlplus::basename_part(
                                                                                         sView_filename) + "_mask",
                                                                                 "png");
                const std::string mask__filename_global = stlplus::create_filespec(sfm_data.s_root_path, "mask", "png");

                Image<unsigned char> imageMask;

                // 局部mask
                if (stlplus::file_exists(mask_filename_local)) {
                    if (!ReadImage(mask_filename_local.c_str(), &imageMask)) {
                        std::cerr << "Invalid mask: " << mask_filename_local << std::endl
                                  << "Stopping feature extraction." << std::endl;
                        preemptive_exit = true;
                        continue;
                    }
                    // 仅当局部mask符合当前图像大小时使用
                    if (imageMask.Width() == imageGray.Width() && imageMask.Height() == imageGray.Height()) {
                        mask = &imageMask;
                    }
                    // 全局mask
                } else {
                    if (stlplus::file_exists(mask__filename_global)) {
                        if (!ReadImage(mask__filename_global.c_str(), &imageMask)) {
                            std::cerr << "Invalid mask: " << mask__filename_global << std::endl
                                      << "Stopping feature extraction." << std::endl;
                            preemptive_exit = true;
                            continue;
                        }
                        // 仅当全局mask符合当前图像大小时使用
                        if (imageMask.Width() == imageGray.Width() && imageMask.Height() == imageGray.Height())
                            mask = &imageMask;
                    }
                }

                // 计算特征和描述子并生成文件
                auto regions = image_describer->Describe(imageGray, mask);
                if (regions && !image_describer->Save(regions.get(), sFeat, sDesc)) {
                    std::cerr << "Cannot save regions for images: " << sView_filename << std::endl
                              << "Stopping feature extraction." << std::endl;
                    preemptive_exit = true;
                    continue;
                }
            }
            ++my_progress_bar;
        }
        std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
    }
    return EXIT_SUCCESS;
}