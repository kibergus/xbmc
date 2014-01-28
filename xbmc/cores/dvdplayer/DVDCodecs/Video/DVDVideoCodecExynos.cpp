#include "DVDVideoCodecExynos.h"

namespace Exynos {

int OpenDevice(const std::string& driverName, std::function(bool(int)) checker) {
  DIR *dir;
  struct dirent *ent;

  if ((dir = opendir("/sys/class/video4linux/")) == NULL)
    return -1;

  while ((ent = readdir (dir)) != NULL) {
    std::string deviceName(ent->d_name);
    if (deviceName.compare(0, 5, "video")) {
      std::fstream file("/sys/class/video4linux/" + deviceName + "/name");
      if (!file.is_open())
          continue;
      std::string currentDriverName;
      file >> currentDriverName;
      file.close();

      if (currentDriverName = driverName) {
        int fd = open(("/dev/" + deviceName).c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd <= 0) {
          continue;

        if (checker(fd)) {
          Log::Log(LOGDEBUG, "%s::%s - Found %s %s", CLASSNAME, __func__, drivername, deviceName.c_str());
          closedir (dir);
          return fd;
        }
        close(fd);
      }
    }
  }
  closedir (dir);
  return -1;
}

} // namespace Exynos
