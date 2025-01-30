#ifndef IO__PIPE_HPP
#define IO__PIPE_HPP
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <stdexcept>

struct DataPacket  // 数据帧格式（定位坐标（x,y,z）,是否启动定位)
{
  double x;
  double y;
  double z;
  bool is_locate;
};
class NamedPipe
{
private:
  std::string pipe_path_;  // 管道名
  int fd_;                 // 管道文件描述符
  bool is_server_;         // 是否为接收端

public:
  // 构造函数，创建有名管道
  NamedPipe(const std::string & pipe_path, bool is_server)
  : pipe_path_(pipe_path), is_server_(is_server), fd_(-1)
  {
    // 创建FIFO文件
    if (is_server) {
      if (mkfifo(pipe_path.c_str(), 0666) == -1 && errno != EEXIST) {
        tools::logger()->warn("Failed to write to FIFO: {}", std::string(strerror(errno)));
        throw std::runtime_error("Failed to open FIFO: " + std::string(strerror(errno)));
      }
    }
  }

  // 打开管道
  void open()
  {
    int flags = is_server_ ? O_RDONLY | O_NONBLOCK : O_WRONLY | O_NONBLOCK;  // 设置为非阻塞模式
    fd_ = ::open(pipe_path_.c_str(), flags);
    if (fd_ == -1) {
      tools::logger()->warn("Failed to write to FIFO: {}", std::string(strerror(errno)));
      throw std::runtime_error("Failed to open FIFO: " + std::string(strerror(errno)));
    }
  }

  // 写数据
  bool write(const void * data, size_t size)
  {
    if (fd_ == -1 || is_server_) return false;
    ssize_t result = ::write(fd_, data, size);
    if (result == -1) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return false;  // 当前没有空间可写
      }
      tools::logger()->warn("Failed to write to FIFO: {}", std::string(strerror(errno)));
      throw std::runtime_error("Failed to open FIFO: " + std::string(strerror(errno)));
    }
    return result == static_cast<ssize_t>(size);
  }

  // 读数据
  bool read(void * buffer, size_t size)
  {
    if (fd_ == -1 || !is_server_) return false;
    ssize_t result = ::read(fd_, buffer, size);
    if (result == -1) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return false;  // 当前没有数据可读
      }
      tools::logger()->warn("Failed to write to FIFO: {}", std::string(strerror(errno)));
      throw std::runtime_error("Failed to open FIFO: " + std::string(strerror(errno)));
      return false;
    }
    return result == static_cast<ssize_t>(size);
  }

  // 关闭管道
  void close()
  {
    if (fd_ != -1) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  // 析构函数
  ~NamedPipe()
  {
    close();
    if (is_server_) {
      unlink(pipe_path_.c_str());  // 删除管道文件
    }
  }
};

#endif  // IO__PIPE_HPP