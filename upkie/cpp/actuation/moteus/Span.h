// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     Copyright 2019-2021 Josh Pieper, jjp@pobox.com.
 *     SPDX-License-Identifier: Apache-2.0
 */

#pragma once

namespace upkie {

namespace moteus {

/*! Fixed-size array with runtime size information.
 *
 * This class assumes the array is allocated and deallocated externally. It
 * stores a pointer to and the size of the allocated memory area.
 */
template <typename T>
class Span {
 public:
  //! Store info to an allocated array
  Span(T* ptr, size_t size) : ptr_(ptr), size_(size) {}

  //! Empty array
  Span() : ptr_(nullptr), size_(0) {}

  //! Get pointer to first element
  T* data() { return ptr_; }

  //! Const-variant of \ref data
  T* data() const { return ptr_; }

  //! Get size of allocated area
  size_t size() const { return size_; }

  //! Check if area is empty
  bool empty() const { return (size_ == 0); }

  //! Get i-th array element
  T& operator[](size_t i) { return ptr_[i]; }

  //! Get i-th array element
  T& operator[](size_t i) const { return ptr_[i]; }

  //! Get pointer to the first element of the array
  T* begin() { return ptr_; }

  //! Get pointer to the last element of the array
  T* end() { return ptr_ + size_; }

  //! Get const pointer to the first element of the array
  const T* begin() const { return ptr_; }

  //! Get const pointer to the last element of the array
  const T* end() const { return ptr_ + size_; }

 private:
  //! Pointer to first element
  T* ptr_;

  //! Size of allocated memory
  size_t size_;
};

}  // namespace moteus

}  // namespace upkie
