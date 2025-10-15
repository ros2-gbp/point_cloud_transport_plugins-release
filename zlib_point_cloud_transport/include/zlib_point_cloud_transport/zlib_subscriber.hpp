// Copyright (c) 2023, Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef ZLIB_POINT_CLOUD_TRANSPORT__ZLIB_SUBSCRIBER_HPP_
#define ZLIB_POINT_CLOUD_TRANSPORT__ZLIB_SUBSCRIBER_HPP_

#include <string>

#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>

#include <point_cloud_transport/simple_subscriber_plugin.hpp>
#include <point_cloud_transport/transport_hints.hpp>

namespace zlib_point_cloud_transport
{

class ZlibSubscriber
  : public point_cloud_transport::SimpleSubscriberPlugin<
    point_cloud_interfaces::msg::CompressedPointCloud2>
{
public:
  std::string getTransportName() const override;

  void declareParameters() override;

  std::string getDataType() const override
  {
    return "point_cloud_interfaces/msg/CompressedPointCloud2";
  }

  DecodeResult decodeTyped(const point_cloud_interfaces::msg::CompressedPointCloud2 & compressed)
  const override;
};
}  // namespace zlib_point_cloud_transport

#endif  // ZLIB_POINT_CLOUD_TRANSPORT__ZLIB_SUBSCRIBER_HPP_
