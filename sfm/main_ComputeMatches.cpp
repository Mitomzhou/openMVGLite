#include "openMVG/graph/graph.hpp"
#include "openMVG/graph/graph_stats.hpp"
#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust_Angular.hpp"
#include "openMVG/matching_image_collection/Eo_Robust.hpp"
#include "openMVG/matching_image_collection/H_ACRobust.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/stl/stl.hpp"
#include "openMVG/system/timer.hpp"

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;
using namespace openMVG::matching_image_collection;
using namespace std;

enum EGeometricModel
{
    FUNDAMENTAL_MATRIX = 0,         // 基本矩阵
    ESSENTIAL_MATRIX   = 1,         // 本质矩阵
    HOMOGRAPHY_MATRIX  = 2,         // 单应矩阵
    ESSENTIAL_MATRIX_ANGULAR = 3,   // 基本矩阵角
    ESSENTIAL_MATRIX_ORTHO = 4,     // 基本正交矩阵
    ESSENTIAL_MATRIX_UPRIGHT = 5    // 基本矩阵直立
};

enum EPairMode
{
    PAIR_EXHAUSTIVE = 0,    // 全部配对
    PAIR_CONTIGUOUS = 1,    // 相邻配对
    PAIR_FROM_FILE  = 2     // 从文件选择配对方法
};


int main()
{
    // 输入文件
    std::string sSfM_Data_Filename = "/home/mitom/3DReconstruction/git/ImageDataset_SceauxCastle/sfm_out/sfm_data.json";
    // 特征文件和描述子文件输出到matches文件夹
    std::string sMatchesDirectory = "/home/mitom/3DReconstruction/git/ImageDataset_SceauxCastle/sfm_out/matches";
    // 几何模型
    std::string sGeometricModel = "f";
    // f距离比
    float fDistRatio = 0.8f;
    // 配对模型
    int iMatchingVideoMode = -1;
    // 从文件的匹配文件的路径
    std::string sPredefinedPairList = "";
    // 匹配器
    std::string sNearestMatchingMethod = "AUTO";
    // 是否重新全部配对，不使用之前已经计算出的成果
    bool bForce = false;
    // 函数Robust_model_estimation中讲解
    bool bGuided_matching = false;
    // funtor GeometricFilter_FMatrix_AC中讲解
    int imax_iteration = 2048;
    // 0->默认区域提供程序（在内存中加载和存储所有区域）other->缓存区域提供程序（按需加载和存储区域）
    unsigned int ui_max_cache_size = 0;

    // 默认是全部匹配
    EPairMode ePairmode = (iMatchingVideoMode == -1 ) ? PAIR_EXHAUSTIVE : PAIR_CONTIGUOUS;

    // 校验
    if (sPredefinedPairList.length()) {
        ePairmode = PAIR_FROM_FILE;
        if (iMatchingVideoMode>0) {
            std::cerr << "\nIncompatible options: --videoModeMatching and --pairList" << std::endl;
            return EXIT_FAILURE;
        }
    }
    if (sMatchesDirectory.empty() || !stlplus::is_folder(sMatchesDirectory))  {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return EXIT_FAILURE;
    }

    // 默认基础矩阵
    EGeometricModel eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
    std::string sGeometricMatchesFilename = "";
    switch (sGeometricModel[0])
    {
        case 'f': case 'F':
            eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
            sGeometricMatchesFilename = "matches.f.bin";
            break;
        case 'e': case 'E':
            eGeometricModelToCompute = ESSENTIAL_MATRIX;
            sGeometricMatchesFilename = "matches.e.bin";
            break;
        case 'h': case 'H':
            eGeometricModelToCompute = HOMOGRAPHY_MATRIX;
            sGeometricMatchesFilename = "matches.h.bin";
            break;
        case 'a': case 'A':
            eGeometricModelToCompute = ESSENTIAL_MATRIX_ANGULAR;
            sGeometricMatchesFilename = "matches.f.bin";
            break;
        case 'o': case 'O':
            eGeometricModelToCompute = ESSENTIAL_MATRIX_ORTHO;
            sGeometricMatchesFilename = "matches.o.bin";
            break;
        case 'u': case 'U':
            eGeometricModelToCompute = ESSENTIAL_MATRIX_UPRIGHT;
            sGeometricMatchesFilename = "matches.f.bin";
            break;
        default:
            std::cerr << "Unknown geometric model" << std::endl;
            return EXIT_FAILURE;
    }

    /*************************************
     特征匹配步骤
     * 1、加载视图图像描述（区域：特征和描述符）
     * 2、计算给定的匹配模式进行特征匹配
     * 3、计算几何关联的特征匹配（基于给定的匹配模式鲁棒估计）
     * 4、导出计算数据
     *************************************/

    // 加载视图
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
        return EXIT_FAILURE;
    }
    // 匹配的描述子初始化
    using namespace openMVG::features;
    const std::string sImage_describer = stlplus::create_filespec(sMatchesDirectory, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type) {
        std::cerr << "Invalid: "  << sImage_describer << " regions type file." << std::endl;
        return EXIT_FAILURE;
    }

    // 计算给定的匹配模式进行特征匹配
    std::shared_ptr<Regions_Provider> regions_provider;
    if (ui_max_cache_size == 0){
        // Default regions provider (load & store all regions in memory)
        regions_provider = std::make_shared<Regions_Provider>();
    }else{
        // Cached regions provider (load & store regions on demand)
        regions_provider = std::make_shared<Regions_Provider_Cache>(ui_max_cache_size);
    }

    // 进度条
    C_Progress_display progress;

    if (!regions_provider->load(sfm_data, sMatchesDirectory, regions_type, &progress)) {
        std::cerr << std::endl << "Invalid regions." << std::endl;
        return EXIT_FAILURE;
    }

    PairWiseMatches map_PutativesMatches;

    // Build some alias from SfM_Data Views data:
    // - List views as a vector of filenames & image sizes
    std::vector<std::string> vec_fileNames;
    std::vector<std::pair<size_t, size_t>> vec_imagesSize;
    {
        vec_fileNames.reserve(sfm_data.GetViews().size());
        vec_imagesSize.reserve(sfm_data.GetViews().size());
        for (Views::const_iterator iter = sfm_data.GetViews().begin(); iter != sfm_data.GetViews().end(); ++iter) {
            const View * v = iter->second.get();
            vec_fileNames.push_back(stlplus::create_filespec(sfm_data.s_root_path,v->s_Img_path));
            vec_imagesSize.push_back(std::make_pair( v->ui_width, v->ui_height) );
        }
    }

    std::cout << std::endl << " - PUTATIVE MATCHES - " << std::endl;

    std::cout << "Use: ";
    switch (ePairmode)
    {
        case PAIR_EXHAUSTIVE: std::cout << "exhaustive pairwise matching" << std::endl; break;
        case PAIR_CONTIGUOUS: std::cout << "sequence pairwise matching" << std::endl; break;
        case PAIR_FROM_FILE:  std::cout << "user defined pairwise matching" << std::endl; break;
    }

    // 根据请求的匹配方法分配正确的匹配器
    std::unique_ptr<Matcher> collectionMatcher;
    if (sNearestMatchingMethod == "AUTO"){
        if (regions_type->IsScalar()){
            std::cout << "Using FAST_CASCADE_HASHING_L2 matcher" << std::endl;
            collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions(fDistRatio));
        }else if (regions_type->IsBinary()){
            std::cout << "Using BRUTE_FORCE_HAMMING matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions(fDistRatio, BRUTE_FORCE_HAMMING));
        }
    }else if (sNearestMatchingMethod == "BRUTEFORCEL2") {// 暴力方法
        std::cout << "Using BRUTE_FORCE_L2 matcher" << std::endl;
        collectionMatcher.reset(new Matcher_Regions(fDistRatio, BRUTE_FORCE_L2));
    }else if (sNearestMatchingMethod == "BRUTEFORCEHAMMING") {
        std::cout << "Using BRUTE_FORCE_HAMMING matcher" << std::endl;
        collectionMatcher.reset(new Matcher_Regions(fDistRatio, BRUTE_FORCE_HAMMING));
    }else if (sNearestMatchingMethod == "HNSWL2") {
        std::cout << "Using HNSWL2 matcher" << std::endl;
        collectionMatcher.reset(new Matcher_Regions(fDistRatio, HNSW_L2));
    }else if (sNearestMatchingMethod == "ANNL2") {
        std::cout << "Using ANN_L2 matcher" << std::endl;
        collectionMatcher.reset(new Matcher_Regions(fDistRatio, ANN_L2));
    }else if (sNearestMatchingMethod == "CASCADEHASHINGL2") {
        std::cout << "Using CASCADE_HASHING_L2 matcher" << std::endl;
        collectionMatcher.reset(new Matcher_Regions(fDistRatio, CASCADE_HASHING_L2));
    }else if (sNearestMatchingMethod == "FASTCASCADEHASHINGL2") {
        std::cout << "Using FAST_CASCADE_HASHING_L2 matcher" << std::endl;
        collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions(fDistRatio));
    }

    if (!collectionMatcher) {
        std::cerr << "Invalid Nearest Neighbor method: " << sNearestMatchingMethod << std::endl;
        return EXIT_FAILURE;
    }

    // 开始匹配
    system::Timer timer;
    {
        // 从匹配模式计算必须匹配的对列表
        Pair_Set pairs;
        switch (ePairmode)
        {
            case PAIR_EXHAUSTIVE: pairs = exhaustivePairs(sfm_data.GetViews().size()); break;
            case PAIR_CONTIGUOUS: pairs = contiguousWithOverlap(sfm_data.GetViews().size(), iMatchingVideoMode); break;
            case PAIR_FROM_FILE:    if (!loadPairs(sfm_data.GetViews().size(), sPredefinedPairList, pairs)){return EXIT_FAILURE;}break;
        }
        // 假设对的照片匹配
        collectionMatcher->Match(regions_provider, pairs, map_PutativesMatches, &progress);
        //---------------------------------------
        //-- 导出假定匹配
        //---------------------------------------
        if (!Save(map_PutativesMatches, std::string(sMatchesDirectory + "/matches.putative.bin")))
        {
            std::cerr
                    << "Cannot save computed matches in: "
                    << std::string(sMatchesDirectory + "/matches.putative.bin");
            return EXIT_FAILURE;
        }
    }
    std::cout << "Task (Regions Matching) done in (s): " << timer.elapsed() << std::endl;

    //-- 导出假定匹配邻接矩阵
    const std::string sOutName = stlplus::create_filespec(sMatchesDirectory, "PutativeAdjacencyMatrix", "svg");
    PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(), map_PutativesMatches, sOutName);
    //-- 一旦计算出假定的图匹配，就导出视图对图
    {
        std::set<IndexT> set_ViewIds;
        std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(),std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
        graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_PutativesMatches));
        graph::exportToGraphvizData(stlplus::create_filespec(sMatchesDirectory, "putative_matches"),putativeGraph);
    }

    // 假定匹配的几何滤波
    // 1、对所需几何模型的对比估计
    // 2、使用对比估计阈值的上限
    std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(new ImageCollectionGeometricFilter(&sfm_data, regions_provider));
    if (filter_ptr){
        system::Timer timer;
        const double d_distance_ratio = 0.6;
        PairWiseMatches map_GeometricMatches;
        switch (eGeometricModelToCompute){
            case HOMOGRAPHY_MATRIX:{
                const bool bGeometric_only_guided_matching = true;
                const double d_distance_ratio =  bGeometric_only_guided_matching ? -1.0 : d_distance_ratio;
                filter_ptr->Robust_model_estimation(GeometricFilter_HMatrix_AC(4.0, imax_iteration),map_PutativesMatches, bGuided_matching, d_distance_ratio, &progress);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();
                }
                break;
            case FUNDAMENTAL_MATRIX:{
                filter_ptr->Robust_model_estimation(GeometricFilter_FMatrix_AC(4.0, imax_iteration),map_PutativesMatches, bGuided_matching, d_distance_ratio, &progress);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();
                }
                break;
            case ESSENTIAL_MATRIX:{
                filter_ptr->Robust_model_estimation( GeometricFilter_EMatrix_AC(4.0, imax_iteration),map_PutativesMatches, bGuided_matching, d_distance_ratio, &progress);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();
                //-- Perform an additional check to remove pairs with poor overlap
                std::vector<PairWiseMatches::key_type> vec_toRemove;
                for (const auto & pairwisematches_it : map_GeometricMatches){
                    const size_t putativePhotometricCount = map_PutativesMatches.find(pairwisematches_it.first)->second.size();
                    const size_t putativeGeometricCount = pairwisematches_it.second.size();
                    const float ratio = putativeGeometricCount / static_cast<float>(putativePhotometricCount);
                    if (putativeGeometricCount < 50 || ratio < .3f)  {
                        // the pair will be removed
                        vec_toRemove.push_back(pairwisematches_it.first);
                    }
                }
                //-- remove discarded pairs
                for (const auto & pair_to_remove_it : vec_toRemove){
                    map_GeometricMatches.erase(pair_to_remove_it);
                }
                }
                break;
            case ESSENTIAL_MATRIX_ANGULAR:{
                filter_ptr->Robust_model_estimation( GeometricFilter_ESphericalMatrix_AC_Angular<false>(4.0, imax_iteration),map_PutativesMatches,
                                                     bGuided_matching, d_distance_ratio, &progress);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();
                }
                break;
            case ESSENTIAL_MATRIX_ORTHO:{
                filter_ptr->Robust_model_estimation( GeometricFilter_EOMatrix_RA(2.0, imax_iteration),map_PutativesMatches, bGuided_matching,
                                                     d_distance_ratio, &progress);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();
                }
                break;
            case ESSENTIAL_MATRIX_UPRIGHT:{
                filter_ptr->Robust_model_estimation(GeometricFilter_ESphericalMatrix_AC_Angular<true>(4.0, imax_iteration),map_PutativesMatches,
                                                    bGuided_matching, d_distance_ratio, &progress);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();
                }
                break;
        }

        // 导出匹配文件
        if (!Save(map_GeometricMatches,std::string(sMatchesDirectory + "/" + sGeometricMatchesFilename))){
            std::cerr << "Cannot save computed matches in: " << std::string(sMatchesDirectory + "/" + sGeometricMatchesFilename);
            return EXIT_FAILURE;
        }

        std::cout << "Task done in (s): " << timer.elapsed() << std::endl;

        // 导出几何视图图形统计信息
        graph::getGraphStatistics(sfm_data.GetViews().size(), getPairs(map_GeometricMatches));

        // 导出邻接矩阵
        std::cout << "\n Export Adjacency Matrix of the pairwise's geometric matches" << std::endl;
        PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),map_GeometricMatches,
                                             stlplus::create_filespec(sMatchesDirectory, "GeometricAdjacencyMatrix", "svg"));

        // 完成几何过滤后导出视图对图
        {
            std::set<IndexT> set_ViewIds;
            std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(), std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
            graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_GeometricMatches));
            graph::exportToGraphvizData( stlplus::create_filespec(sMatchesDirectory, "geometric_matches"), putativeGraph);
        }
    }

    return EXIT_SUCCESS;
}