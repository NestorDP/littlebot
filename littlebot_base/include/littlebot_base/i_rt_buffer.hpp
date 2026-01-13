#pragma once

namespace littlebot_base
{

template<typename T>
class IRTBuffer
{
public:
  virtual ~IRTBuffer() = default;

  // RT context
  virtual const T* readRT() const noexcept = 0;

  // Non-RT context
  virtual void writeNonRT(const T& data) = 0;
};

}  // namespace littlebot_base