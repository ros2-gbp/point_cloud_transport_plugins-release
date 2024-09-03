/*
 * Copyright (c) <Current Year>, <Your Name Here> (if desired)
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <template_point_cloud_transport/template_subscriber.hpp>

namespace template_point_cloud_transport
{
void TemplateSubscriber::declareParameters()
{
  // Although not required, it is often useful to expose static or dynamically configurable
  // parameters to control your compression algorithm speed and output quality.
}

std::string TemplateSubscriber::getTransportName() const
{
  // This should match the name of your transport's prefix.
  // e.g. for template_point_cloud_transport, the prefix is template
  return "template";
}

TemplateSubscriber::DecodeResult TemplateSubscriber::decodeTyped(
  const point_cloud_interfaces::msg::CustomMessage & msg) const
{
  auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();

  // Add your decompression code here!
  // turtle.decompress(msg, result);

  // Although not required, it is often convenient to implement your decompression 
  // algo in a separate file and call it here. This keeps your code 
  // clean and easy to read.  

  return result;
}

}  // namespace template_point_cloud_transport
