/*
 * Copyright (C) 2022 Can-lab Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#pragma once

#include <system_error>

namespace clpe
{
static const std::error_code kNoError(0, std::generic_category());

class ConnectionError : public std::error_category
{
public:
  static constexpr int CannotProbeDriver = -1;
  static constexpr int CannotFindNetwork = -2;
  static constexpr int CannotSetAddress = -3;
  static constexpr int CannotPing = -4;
  static constexpr int CannotCreateSocket = -5;
  static constexpr int CannotConnectSocket = -6;

  static const ConnectionError& get()
  {
    static const ConnectionError inst;
    return inst;
  }

  const char* name() const noexcept override
  {
    return "ConnectionError";
  }

  std::string message(int code) const override
  {
    switch (code)
    {
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

class StartStreamError : public std::error_category
{
public:
  static constexpr int FailToCreateTask = -1;

  static const StartStreamError& get()
  {
    static const StartStreamError inst;
    return inst;
  }

  const char* name() const noexcept override
  {
    return "StartStreamError";
  }

  std::string message(int code) const override
  {
    switch (code)
    {
      case FailToCreateTask:
        return "failed to create task";
      default:
        return std::to_string(code);
    }
  }
};

class GetFrameError : public std::error_category
{
public:
  static constexpr int FrameNotReady = -2;
  static constexpr int InvalidCamId = -3;

  static const GetFrameError& get()
  {
    static const GetFrameError inst;
    return inst;
  }

  const char* name() const noexcept override
  {
    return "GetFrameError";
  }

  std::string message(int code) const override
  {
    switch (code)
    {
      case FrameNotReady:
        return "frame not ready";
      case InvalidCamId:
        return "invalid camera id";
      default:
        return std::to_string(code);
    }
  }
};

class GetEepromDataError : public std::error_category
{
public:
  static constexpr int InvalidCamId = -1;
  static constexpr int FailedCommunication = -3;
  static constexpr int BadChecksum = -4;

  static const GetEepromDataError& get()
  {
    static const GetEepromDataError inst;
    return inst;
  }

  const char* name() const noexcept override
  {
    return "GetEepromDataError";
  }

  std::string message(int code) const override
  {
    switch (code)
    {
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
}  // namespace clpe
