/*
    Profile.cpp

    Copyright © 2022 MSAD Mode2P. All rights reserved.
*/
#include "Profile.hpp"
#include "appusr.hpp"

#include <fstream>
#include <glob.h>

Profile::Profile(const std::string& paths) {
  glob_t globbuf;
  glob (paths.c_str(), 0, NULL, &globbuf);
  for (int i = 0; i < globbuf.gl_pathc; i++) {
    std::string path = globbuf.gl_pathv[i];

  std::ifstream is(path);
  std::string key, value;
  std::string delim = "=";
  size_t pos;

  if (!is.is_open()) {
    _log("open failed %s", path.c_str());
  } else {
    _log("reading profile %s", path.c_str());
    for (std::string line; std::getline(is, line);) {
      if ( (pos = line.find(delim)) != std::string::npos ) {
	key = line.substr(0,pos);
	value = line.substr(pos+delim.length(),line.length());
	if (profile.find(key) != profile.end() ) {
	  _log("*** WARNING - profile key %s duplication detected in %s but overwritten", key.c_str(), path.c_str());
	}
	profile[key] = value;
	_log("%s = %s", key.c_str(), value.c_str());
      }
    }
  }
  }
  globfree(&globbuf);
}

std::string Profile::getValueAsStr(const std::string& key) {
  if ( profile.find(key) == profile.end() ) {
    _log("*** WARNING - profile key %s not exist. empty string used as default", key.c_str());
    profile[key] = "";
  }
  return profile[key];
}

double Profile::getValueAsNum(const std::string& key) {
  if ( profile.find(key) == profile.end() ) {
    _log("*** WARNING - profile key %s not exist. zero value used as default", key.c_str());
    profile[key] = "0.0";
  }
  return std::stod(profile[key]);
}
