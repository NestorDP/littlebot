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

/**
 * @file test_main.cpp
 * @brief Main entry point for all unit tests
 * @author Nestor Neto
 * @date 2024
 */

#include <gtest/gtest.h>

/**
 * @brief Main function that runs all unit tests
 *
 * This is the entry point for running all Google Test unit tests.
 * It initializes the Google Test framework and runs all registered tests.
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Test result (0 for success, non-zero for failure)
 */
int main(int argc, char **argv)
{
  // Initialize Google Test
  ::testing::InitGoogleTest(&argc, argv);

  // Run all tests
  return RUN_ALL_TESTS();
}
