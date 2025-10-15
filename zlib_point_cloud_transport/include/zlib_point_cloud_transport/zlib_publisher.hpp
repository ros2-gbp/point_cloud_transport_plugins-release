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


#ifndef ZLIB_POINT_CLOUD_TRANSPORT__ZLIB_PUBLISHER_HPP_
#define ZLIB_POINT_CLOUD_TRANSPORT__ZLIB_PUBLISHER_HPP_

#include <memory>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>

#include <point_cloud_transport/simple_publisher_plugin.hpp>
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>


namespace zlib_point_cloud_transport
{

class ZlibPublisher
  : public point_cloud_transport::SimplePublisherPlugin<
    point_cloud_interfaces::msg::CompressedPointCloud2>
{
public:
  std::string getTransportName() const override;

  void declareParameters(const std::string & base_topic) override;

  std::string getDataType() const override
  {
    return "point_cloud_interfaces/msg/CompressedPointCloud2";
  }

  TypedEncodeResult encodeTyped(const sensor_msgs::msg::PointCloud2 & raw) const override;

private:
  int encode_level_{7};
};
}  // namespace zlib_point_cloud_transport

#endif  // ZLIB_POINT_CLOUD_TRANSPORT__ZLIB_PUBLISHER_HPP_
