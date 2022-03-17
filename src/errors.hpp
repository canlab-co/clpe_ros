#include <system_error>

namespace clpe
{
class ConnectionError : public std::error_code
{
public:
  static constexpr int CannotProbeDriver = -1;
  static constexpr int CannotFindNetwork = -2;
  static constexpr int CannotSetAddress = -3;
  static constexpr int CannotPing = -4;
  static constexpr int CannotCreateSocket = -5;
  static constexpr int CannotConnectSocket = -6;

  class ErrorCategory : public std::error_category
  {
    const char * name() const noexcept override { return "ConnectionError"; }

    std::string message(int code) const override
    {
      switch (code) {
        case CannotProbeDriver:
          return "cannot probe driver";
        case CannotFindNetwork:
          return "cannot find network";
        case CannotSetAddress:
          return "cannot set address";
        case CannotPing:
          return "cannot ping";
        case CannotCreateSocket:
          return "cannot create socket";
        case CannotConnectSocket:
          return "cannot connect socket";
        default:
          return std::to_string(code);
      }
    }
  };

  ConnectionError(int code) : std::error_code(code, ErrorCategory()) {}
};

class StartStreamError : public std::error_code
{
public:
  static constexpr int FailToCreateTask = -1;

  class ErrorCategory : public std::error_category
  {
    const char * name() const noexcept override { return "StartStreamError"; }

    std::string message(int code) const override
    {
      switch (code) {
        case FailToCreateTask:
          return "failed to create task";
        default:
          return std::to_string(code);
      }
    }
  };

  StartStreamError(int code) : std::error_code(code, ErrorCategory()) {}
};

class GetFrameError : public std::error_code
{
public:
  static constexpr int FrameNotReady = -2;
  static constexpr int InvalidCamId = -3;

  class ErrorCategory : public std::error_category
  {
    const char * name() const noexcept override { return "GetFrameError"; }

    std::string message(int code) const override
    {
      switch (code) {
        case FrameNotReady:
          return "frame not ready";
        case InvalidCamId:
          return "invalid camera id";
        default:
          return std::to_string(code);
      }
    }
  };

  GetFrameError(int code) : std::error_code(code, ErrorCategory()) {}
};

class GetEepromdataError : public std::error_code
{
public:
  static constexpr int InvalidCamId = -1;
  static constexpr int FailedCommunication = -3;
  static constexpr int BadChecksum = -4;

  class ErrorCategory : public std::error_category
  {
    const char * name() const noexcept override { return "GetEepromdataError"; }

    std::string message(int code) const override
    {
      switch (code) {
        case InvalidCamId:
          return "invalid camera id";
        case FailedCommunication:
          return "read failed";
        case BadChecksum:
          return "bad checksum";
        default:
          return std::to_string(code);
      }
    }
  };

  GetEepromdataError(int code) : std::error_code(code, ErrorCategory()) {}
};
}  // namespace clpe
