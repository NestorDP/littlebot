// @ Copyright 2025 Nestor Neto
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

TEST_F(TestGenericSystem, load_generic_system_2dof) {
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_ +
    ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(TestableResourceManager rm(urdf));
}  // NOLINT
