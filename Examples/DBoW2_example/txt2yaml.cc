
#include <iostream>
#include <sys/time.h>

// Opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

// DBoW2
#include <Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h>
#include <Thirdparty/DBoW2/DBoW2/FORB.h>

#define VERBOSE

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

int main(int argc, char** argv)
{

    if (argc != 3)
    {
        std::cout << "txt2yaml <input_path_to_voc_txt> <output_voc_yaml>" << std::endl;
        return 1;
    }

    // load txt vocabulary
    ORBVocabulary voc;
    if (!voc.loadFromTextFile(argv[1]))
    {
        std::cerr << "Failed to load the vocabulary : " << argv[1] << std::endl;
        return 1;
    }

#ifdef VERBOSE
    // Vocabulary info
    std::cout << "Vocabulary info : " << std::endl;
    std::cout << " - branching factor : " << voc.getBranchingFactor() << std::endl;
    std::cout << " - depth levels : " << voc.getDepthLevels() << std::endl;
    std::cout << " - effective levels : " << voc.getEffectiveLevels() << std::endl;
    std::cout << " - size : " << voc.size() << std::endl;
#endif

    cv::FileStorage fs(argv[2], cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
    voc.save(fs);

    return 0;

}
