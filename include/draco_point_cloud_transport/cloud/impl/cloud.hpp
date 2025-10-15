// Copyright (c) 2023, Open Source Robotics Foundation, Inc.
// Copyright (c) 2023, Czech Technical University in Prague
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


/**
 * \file
 * \brief Utilities for comfortable working with PointCloud2 messages.
 * \author Martin Pecka
 */

#ifndef DRACO_POINT_CLOUD_TRANSPORT__CLOUD__IMPL__CLOUD_HPP_
#define DRACO_POINT_CLOUD_TRANSPORT__CLOUD__IMPL__CLOUD_HPP_

#include <string>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace cras
{

namespace impl
{

/**
 * \brief Base of a generic cloud iterator which can return the data in the raw type.
 * \tparam T Type of the value on which the child class will be templated.
 * \tparam TT Type of the value to be retrieved (same as T except for constness).
 * \tparam U Type of the raw data in PointCloud2 (only uchar and const uchar are supported).
 * \tparam C Type of the pointcloud to intialize from (const or not).
 * \tparam V The derived class (yop, curiously recurring template pattern).
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
class GenericCloudIteratorBase : public ::sensor_msgs::impl::PointCloud2IteratorBase<T, TT, U, C, V>
{
public:
  /**
   * \param[in] cloud_msg The PointCloud2 to iterate upon.
   * \param[in] field_name The field to iterate upon.
   */
  GenericCloudIteratorBase(C & cloudMsg, const ::std::string & fieldName);

  /**
   * \brief Get the byte size of the field which this iterator iterates.
   * \return Size in bytes.
   */
  inline size_t getFieldSize() const {return this->fieldSize;}

  /**
   * \brief Return a pointer to the raw data of this field. Only `getFieldSize()` bytes after this pointer are valid.
   * \return The pointer.
   */
  U * rawData() const;

protected:
  //! \brief The byte size of the field which this iterator iterates.
  size_t fieldSize = 0;
};

/**
 * \brief Generic const cloud iterator which can return the data in the raw type.
 */
template<typename T = unsigned char>
class GenericCloudConstIterator
  : public GenericCloudIteratorBase<T, const T, const unsigned char,
    const ::sensor_msgs::msg::PointCloud2, GenericCloudConstIterator>
{
public:
  /**
   * \param[in] cloud_msg The PointCloud2 to iterate upon.
   * \param[in] field_name The field to iterate upon.
   */
  GenericCloudConstIterator(
    const ::sensor_msgs::msg::PointCloud2 & cloud_msg,
    const ::std::string & field_name)
  : GenericCloudIteratorBase<T, const T, const unsigned char, const ::sensor_msgs::msg::PointCloud2,
      GenericCloudConstIterator>::GenericCloudIteratorBase(cloud_msg, field_name)
  {
  }

  /**
   * \brief Return the raw data converted to the given datatype. The datatype has to have the same size as the field.
   * \tparam D Type of the data the field should be converted to.
   * \return Pointer to the current raw data converted to the given datatype.
   * \throws std::runtime_error If sizeof(D) is not the same as getFieldSize().
   */
  template<typename D>
  const D * dataAs() const
  {
    if (sizeof(D) != this->getFieldSize()) {
      throw ::std::runtime_error(
              "Cannot convert field of size " + ::std::to_string(this->getFieldSize()) +
              " to a type of size " + ::std::to_string(sizeof(D)));
    }
    return reinterpret_cast<const D *>(this->rawData());
  }
};

/**
 * \brief Generic non-const cloud iterator which can return the data in the raw type.
 */
template<typename T = unsigned char>
class GenericCloudIterator
  : public GenericCloudIteratorBase<T, T, unsigned char, ::sensor_msgs::msg::PointCloud2,
    GenericCloudIterator>
{
public:
  /**
   * \param[in] cloud_msg The PointCloud2 to iterate upon.
   * \param[in] field_name The field to iterate upon.
   */
  GenericCloudIterator(
    ::sensor_msgs::msg::PointCloud2 & cloud_msg,
    const ::std::string & field_name)
  : GenericCloudIteratorBase<T, T, unsigned char, ::sensor_msgs::msg::PointCloud2,
      GenericCloudIterator>::GenericCloudIteratorBase(cloud_msg, field_name)
  {
  }

  /**
   * \brief Return the raw data converted to the given datatype. The datatype has to have the same size as the field.
   * \tparam D Type of the data the field should be converted to.
   * \return Pointer to the current raw data converted to the given datatype.
   * \throws std::runtime_error If sizeof(D) is not the same as getFieldSize().
   */
  template<typename D>
  D * dataAs() const
  {
    if (sizeof(D) != this->getFieldSize()) {
      throw ::std::runtime_error(
              "Cannot convert field of size " + ::std::to_string(this->getFieldSize()) +
              " to a type of size " + ::std::to_string(sizeof(D)));
    }
    return reinterpret_cast<D *>(this->rawData());
  }

  /**
   * \brief Copy all values of this field from another iterator.
   * \param otherIter The other iterator.
   */
  void copyData(const ::cras::impl::GenericCloudConstIterator<T> & otherIter) const;

  /**
   * \brief Copy all values of this field from another iterator.
   * \param otherIter The other iterator.
   */
  void copyData(const ::cras::impl::GenericCloudIterator<T> & otherIter) const;
};

}  // namespace impl
}  // namespace cras

#endif  // DRACO_POINT_CLOUD_TRANSPORT__CLOUD__IMPL__CLOUD_HPP_
