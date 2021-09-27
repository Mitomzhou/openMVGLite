#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"
#include "openMVG/sfm/pipelines/global/GlobalSfM_rotation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/GlobalSfM_translation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::sfm;

int main()
{
    // sfm_data.json路径
    std::string sSfM_Data_Filename = "/home/mitom/3DReconstruction/git/ImageDataset_SceauxCastle/sfm_out/sfm_data.json";
    // 几何匹配的路径
    std::string sMatchesDir = "/home/mitom/3DReconstruction/git/ImageDataset_SceauxCastle/sfm_out/matches";
    // 匹配的文件名
    std::string sMatchFilename = "matches.f.bin";
    // sfm输出路径
    std::string sOutDir = "/home/mitom/3DReconstruction/git/ImageDataset_SceauxCastle/sfm_out";
    // 预先定义motion
    bool b_use_motion_priors = false;
    //参数：设置旋转平均法
    int iRotationAveragingMethod = int (ROTATION_AVERAGING_L2);
    //参数：设置转换平均法
    int iTranslationAveragingMethod = int (TRANSLATION_AVERAGING_SOFTL1);

    // NONE                     固有参数将被视为固定参数
    // ADJUST_FOCAL_LENGTH      焦距将被视为细化的变量
    // ADJUST_PRINCIPAL_POINT   将主点视为细化变量
    // ADJUST_DISTORTION        畸变参数将被视为细化的变量
    // ADJUST_ALL               所有参数将被视为细化变量
    // 参数： 内在参数，用于控制哪一个摄像机参数必须被视为保持不变的变量以进行非线性细化的类型
    std::string sIntrinsic_refinement_options = "ADJUST_ALL";

    // 校验
    if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
        iRotationAveragingMethod > ROTATION_AVERAGING_L2 )  {
        std::cerr << "\n Rotation averaging method is invalid" << std::endl;
        return EXIT_FAILURE;
    }

    // 设置相机内参矩阵并校验
    const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options = cameras::StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);
    if (intrinsic_refinement_options == static_cast<cameras::Intrinsic_Parameter_Type>(0) ){
        std::cerr << "Invalid input for Bundle Adjusment Intrinsic parameter refinement option" << std::endl;
        return EXIT_FAILURE;
    }

    if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 || iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1 )  {
        std::cerr << "\n Translation averaging method is invalid" << std::endl;
        return EXIT_FAILURE;
    }

    // 加载视图
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl  << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
        return EXIT_FAILURE;
    }

    // 从图像描述符文件初始化区域类型（用于图像区域提取）
    using namespace openMVG::features;
    const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type){
        std::cerr << "Invalid: " << sImage_describer << " regions type file." << std::endl;
        return EXIT_FAILURE;
    }

    // 特征读取
    std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
    if (!feats_provider->load(sfm_data, sMatchesDir, regions_type)) {
        std::cerr << std::endl << "Invalid features." << std::endl;
        return EXIT_FAILURE;
    }

    // 匹配文件读取 eg. 默认：matches.f.bin
    std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
    if(!(matches_provider->load(sfm_data, sMatchFilename) ||
              matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.f.txt")) ||
              matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.f.bin")))){
        std::cerr << std::endl << "Invalid matches file." << std::endl;
        return EXIT_FAILURE;
    }

    // 输出路径校验
    if (sOutDir.empty())  {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return EXIT_FAILURE;
    }
    if (!stlplus::folder_exists(sOutDir)) {
        if (!stlplus::folder_create(sOutDir)){
            std::cerr << "\nCannot create the output directory" << std::endl;
        }
    }

    // 全局SfM重建过程
    openMVG::system::Timer timer;
    GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(sfm_data, sOutDir, stlplus::create_filespec(sOutDir, "Reconstruction_Report.html"));

    // 配置特征与匹配
    sfmEngine.SetFeaturesProvider(feats_provider.get());
    sfmEngine.SetMatchesProvider(matches_provider.get());

    // 配置重建参数
    sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    sfmEngine.Set_Use_Motion_Prior(b_use_motion_priors);

    // 配置全局运动平均方法
    sfmEngine.SetRotationAveragingMethod(ERotationAveragingMethod(iRotationAveragingMethod));
    sfmEngine.SetTranslationAveragingMethod(ETranslationAveragingMethod(iTranslationAveragingMethod));

    // BA开始
    if (sfmEngine.Process()){
        std::cout << std::endl << " Total Ac-Global-Sfm took (s): " << timer.elapsed() << std::endl;
        // html报告输出
        std::cout << "...Generating SfM_Report.html" << std::endl;
        Generate_SfM_Report(sfmEngine.Get_SfM_Data(), stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));

        // 导出计算场景（数据和可视化结果）
        std::cout << "...Export SfM_Data to disk." << std::endl;
        Save(sfmEngine.Get_SfM_Data(), stlplus::create_filespec(sOutDir, "sfm_data", ".bin"), ESfM_Data(ALL));
        Save(sfmEngine.Get_SfM_Data(),stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"), ESfM_Data(ALL));

        return EXIT_SUCCESS;
    }
    return EXIT_SUCCESS;
}