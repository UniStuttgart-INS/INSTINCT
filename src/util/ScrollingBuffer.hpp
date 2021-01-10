/// @file ScrollingBuffer.hpp
/// @brief
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-09

#pragma once

#include <vector>

namespace NAV
{
template<class T>
class ScrollingBuffer
{
  public:
    ScrollingBuffer(size_t maxSize = 2000)
        : maxSize(maxSize)
    {
        data_.reserve(maxSize);
    }

    void AddValue(T value)
    {
        if (data_.size() < maxSize)
        {
            data_.push_back(value);
        }
        else
        {
            data_.at(offset_) = value;
            offset_ = (offset_ + 1) % maxSize;
        }
    }
    void clear()
    {
        if (data_.size() > 0)
        {
            data_.clear();
            offset_ = 0;
        }
    }

    // TODO: Resize

    T back()
    {
        if (data_.size() < maxSize || offset_ == 0)
        {
            return data_.back();
        }
        else
        {
            return data_.at(offset_ - 1);
        }
    }

    T front()
    {
        if (data_.size() < maxSize)
        {
            return data_.front();
        }
        else
        {
            return data_.at(offset_);
        }
    }

    [[nodiscard]] int offset() const
    {
        return static_cast<int>(offset_);
    }

    [[nodiscard]] const std::vector<T>& data()
    {
        return data_;
    }

  private:
    size_t maxSize;
    size_t offset_ = 0;
    std::vector<T> data_;
};

} // namespace NAV