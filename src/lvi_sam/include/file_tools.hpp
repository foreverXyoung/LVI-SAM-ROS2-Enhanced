#pragma once

/////////////////////////////////// SC-Tools ///////////////////////////////////
#include "sc/Scancontext.h"
#include <fstream>
#include <iomanip>

enum class SCInputType { SINGLE_SCAN_FULL, SINGLE_SCAN_FEAT, MULTI_SCAN_FEAT };

inline void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ") {
    // delimiter: ", " or " " etc.
    int precision = 3;  // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");

    std::ofstream file(fileName);
    if (file.is_open()) {
        file << matrix.format(the_format);
        file.close();
    }
}

inline std::string padZeros(int val, int num_digits = 6) {
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

// zxl(已完成)
inline void loadSCD(std::string fileName, Eigen::MatrixXd& matrix, char delimiter = ' ') {
    // delimiter: ", " or " " etc.
    std::vector<double> matrixEntries;
    std::ifstream matrixDataFile(fileName);
    if (!matrixDataFile.is_open()) {
        std::cout << "读入SCD文件失败!!" << std::endl;
        return;
    }

    std::string matrixRowString;
    std::string matrixEntry;
    int matrixRowNumber = 0;
    while (getline(matrixDataFile, matrixRowString)) {
        std::stringstream matrixRowStringStream(matrixRowString);

        while (getline(matrixRowStringStream, matrixEntry, delimiter)) {
            matrixEntries.push_back(stod(matrixEntry));
        }
        matrixRowNumber++;
    }

    matrix = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber,
                                                                                                matrixEntries.size() / matrixRowNumber);
    matrixDataFile.close();
}

// zxl(已完成)
inline void loadPoses(std::string fileName, Eigen::MatrixXd& matrixPose, char delimiter = ' ') {
    // delimiter: ", " or " " etc.
    std::vector<double> matrixEntries;
    std::ifstream matrixDataFile(fileName);
    if (!matrixDataFile.is_open()) {
        std::cout << "读入SCD文件失败!!" << std::endl;
        return;
    }

    std::string matrixRowString;
    std::string matrixEntry;
    int matrixRowNumber = 0;
    while (getline(matrixDataFile, matrixRowString)) {
        std::stringstream matrixRowStringStream(matrixRowString);

        while (getline(matrixRowStringStream, matrixEntry, delimiter)) {
            matrixEntries.push_back(stod(matrixEntry));
        }
        matrixRowNumber++;
    }

    matrixPose = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber,
                                                                                                    matrixEntries.size() / matrixRowNumber);
    matrixDataFile.close();
}

/////////////////////////////////// File-Tools ///////////////////////////////////
#include <sys/stat.h>  // mkdir, stat
#include <cerrno>
#include <cstring>

inline bool createDirectoryIfNotExists(const std::string& path) {
    if (path.empty()) {
        std::cerr << "Cannot create an empty directory path" << std::endl;
        return false;
    }

    // mkdir(2) only creates one level. Walk each component so paths such as
    // ~/.ros/return_station/gazebo_map also work on a clean machine.
    std::string current;
    std::size_t component_begin = 0;
    if (path.front() == '/') {
        current = "/";
        component_begin = 1;
    }

    while (component_begin <= path.size()) {
        const std::size_t separator = path.find('/', component_begin);
        const std::size_t component_end =
            separator == std::string::npos ? path.size() : separator;
        const std::string component =
            path.substr(component_begin, component_end - component_begin);

        if (!component.empty()) {
            if (!current.empty() && current.back() != '/') {
                current += '/';
            }
            current += component;

            struct stat info;
            if (stat(current.c_str(), &info) != 0) {
                if (errno != ENOENT || mkdir(current.c_str(), 0755) != 0) {
                    std::cerr << "Error creating directory " << current << ": "
                              << std::strerror(errno) << std::endl;
                    return false;
                }
            } else if (!S_ISDIR(info.st_mode)) {
                std::cerr << "Path exists but is not a directory: " << current
                          << std::endl;
                return false;
            }
        }

        if (separator == std::string::npos) {
            break;
        }
        component_begin = separator + 1;
    }

    return true;
}
