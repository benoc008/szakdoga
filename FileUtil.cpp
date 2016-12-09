#include "FileUtil.hpp"

std::vector<std::string> getFilesInDir(std::string dir) {
    std::vector<std::string> ret;

    boost::filesystem::path boostDir(dir);
    boost::filesystem::directory_iterator end_iter;

    if (boost::filesystem::exists(boostDir) && boost::filesystem::is_directory(boostDir)) {
        for (boost::filesystem::directory_iterator dir_iter(boostDir); dir_iter != end_iter; ++dir_iter) {
            if (boost::filesystem::is_regular_file(dir_iter->status())) {
                ret.push_back((*dir_iter).path().generic_string());
            }
        }
    }

    std::sort(ret.begin(), ret.end());
    return ret;
}

bool isDirectory(std::string dir){
    boost::filesystem::path boostDir(dir);
    return boost::filesystem::exists(boostDir) && boost::filesystem::is_directory(boostDir);
}
