
#include <iostream>
#include <sys/time.h>
#include <sys/types.h>
#include <dirent.h>

// Opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

// DBoW2
#include <Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h>
#include <Thirdparty/DBoW2/DBoW2/FORB.h>

// ORB_SLAM2
#include "ORBextractor.h"
#include "Converter.h"


#define VERBOSE

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

int main(int argc, char** argv)
{

    if (argc != 3)
    {
        std::cout << "demo <images_input_folder> <output_voc_file_path>" << std::endl;
        return 1;
    }

    std::vector<std::string> inputFiles;
    // Get image files
    {
        DIR * dir;
        struct dirent *ent;
        if ((dir = opendir(argv[1])) != 0)
        {
            size_t count=0;
            inputFiles.resize(10);
            while ((ent = readdir(dir)) != NULL)
            {

                // if jpg or png append
                std::string fname(ent->d_name);

                if (fname.size() > 3)
                {
                    std::string ext = fname.substr(fname.size()-3, 3);

                    if (ext == "png"|| ext == "jpg")
                    {
                        inputFiles[count] = argv[1] + std::string("/") + fname;
                        count++;
                        if (count >= inputFiles.size())
                        {
                            inputFiles.resize((int)(3*inputFiles.size()/2));
                        }
                    }
                }

            }
            closedir(dir);
            inputFiles.resize(count);
        }
        else
        {
            std::cerr << "Input image folder : " << argv[1] << " is not found" << std::endl;
            return 1;
        }

    }

    // Get features from training images
    std::vector<std::vector<DBoW2::FORB::TDescriptor> > trainingFeatures;
    {
        int nfeatures = 1000;
        float scaleFactor = 1.2;
        int nlevels = 8;
        int iniThFAST = 20;
        int minThFAST = 7;
        ORB_SLAM2::ORBextractor extractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);

        for (size_t i=0; i<inputFiles.size(); i++)
        {
            cv::Mat img = cv::imread(inputFiles[i], cv::IMREAD_GRAYSCALE);
            if (img.empty())
            {
                std::cerr << "Failed to read image : " << inputFiles[i] << std::endl;
                continue;
            }

            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            extractor(img, cv::Mat(), keypoints, descriptors);
            trainingFeatures.push_back(ORB_SLAM2::Converter::toDescriptorVector(descriptors));

#ifdef VERBOSE2
            cv::Mat imageWithKeypoints;
            cv::drawKeypoints(img, keypoints, imageWithKeypoints);
            cv::imshow("Image with keypoints", imageWithKeypoints);
            std::cout << "Keypoints size : " << keypoints.size() << std::endl;
            std::cout << "Descriptor size : " << descriptors.rows << " x " << descriptors.cols << std::endl;
            cv::waitKey(0);
#endif
        }
    }

#ifdef VERBOSE
    std::cout << "----- Training features info : -----" << std::endl;
    std::cout << " - size : " << trainingFeatures.size() << std::endl;
    for (size_t i=0; i<trainingFeatures.size(); i++)
    {
        std::cout << " -- feature " << i << " : nb of TDescriptor = " << trainingFeatures[i].size() << std::endl;
        bool isLarge = (size_t)10 < trainingFeatures[i].size();
        for (size_t j=0; j<std::min((size_t)10, trainingFeatures[i].size()); j++)
        {
            std::cout << "set " << j << " : {" << trainingFeatures[i][j].cols << "} | ";
        }
        if (isLarge)
        {
            std::cout << "... | ";
            size_t j = trainingFeatures[i].size() - 1;
            std::cout << "set " << j << " : {" << trainingFeatures[i][j].cols << "} | ";
        }
        std::cout << std::endl;
    }
    std::cout << "----- Training features info [END] -----" << std::endl;
#endif


    // create a vocabulary
    int k = 8; // branching factor
    int L = 4; // depth levels
    DBoW2::WeightingType weighting = DBoW2::TF_IDF;
    DBoW2::ScoringType scoring = DBoW2::L1_NORM;
    ORBVocabulary voc(k, L, weighting, scoring);
    voc.create(trainingFeatures);

    // Vocabulary info
    std::cout << "Vocabulary info : " << std::endl;
    std::cout << " - branching factor : " << voc.getBranchingFactor() << std::endl;
    std::cout << " - depth levels : " << voc.getDepthLevels() << std::endl;
    std::cout << " - effective levels : " << voc.getEffectiveLevels() << std::endl;
    std::cout << " - nb of words : " << voc.size() << " < k**L = " << (int) std::pow(k, L) << std::endl;

//    voc.



//    {
//        std::vector<DBoW2::WordId> words;
//        voc.getWordsFromNode(0, words);
//        std::cout << "words size : " << words.size() << std::endl;
//    }

//    cv::FileStorage fs(argv[2], cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
//    voc.save(fs);



    // save the vocabulary
    // load the vocabulary



    return 0;

}
