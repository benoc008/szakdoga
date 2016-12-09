#ifndef FILEUTIL_HPP
#define FILEUTIL_HPP

#include <string>
#include <vector>
#include <algorithm>
#include <boost/filesystem.hpp>

std::vector<std::string> getFilesInDir(std::string dir);
bool isDirectory(std::string dir);

#endif //FILEUTIL_HPP
