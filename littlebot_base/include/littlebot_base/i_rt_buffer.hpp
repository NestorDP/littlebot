// @ Copyright 2026 Nestor Neto
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
