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

#define UNIT_TEST

#include <gtest/gtest.h>

#include "littlebot_base/serial_port.hpp"

std::vector<uint8_t> toBytes(const std::string & str)
{
  return std::vector<uint8_t>(str.begin(), str.end());
}

TEST(SerialPortTest, ExtractsValidFrame)
{
  littlebot_base::SerialPort port;

  std::vector<uint8_t> payload;
  port.injectRxData("[TEST_DATA]\n");

  int n = port.read(payload);

  ASSERT_GT(n, 0);
  EXPECT_EQ(payload, toBytes("TEST_DATA"));
}

TEST(SerialPortTest, DiscardsGarbageBeforeFrame)
{
  littlebot_base::SerialPort port;

  std::vector<uint8_t> payload;
  port.injectRxData("xxxxxx[HELLO]\n");

  int n = port.read(payload);

  ASSERT_GT(n, 0);
  EXPECT_EQ(payload, toBytes("HELLO"));
}

TEST(SerialPortTest, DoesNotExtractPartialFrame)
{
  littlebot_base::SerialPort port;

  std::vector<uint8_t> payload;
  port.injectRxData("[PARTIAL");

  int n = port.read(payload);

  EXPECT_EQ(n, 0);
}

TEST(SerialPortTest, ExtractsFrameSplitAcrossReads)
{
  littlebot_base::SerialPort port;
  std::vector<uint8_t> payload;

  port.injectRxData("[SPLIT");
  EXPECT_EQ(port.read(payload), 0);

  port.injectRxData("_FRAME]\n");
  EXPECT_GT(port.read(payload), 0);

  EXPECT_EQ(payload, toBytes("SPLIT_FRAME"));
}

TEST(SerialPortTest, ExtractsMultipleFramesSequentially)
{
  littlebot_base::SerialPort port;
  std::vector<uint8_t> payload;

  port.injectRxData("[ONE]\n[TWO]\n");

  ASSERT_GT(port.read(payload), 0);
  EXPECT_EQ(payload, toBytes("ONE"));

  ASSERT_GT(port.read(payload), 0);
  EXPECT_EQ(payload, toBytes("TWO"));
}

// TEST(SerialPortTest, ThrowsOnMalformedFrame)
// {
//   littlebot_base::SerialPort port;
//   std::vector<uint8_t> payload;

//   port.injectRxData("[BAD]");

//   EXPECT_THROW(
//     port.read(payload),
//     std::runtime_error
//   );
// }

TEST(SerialPortTest, IgnoresLeadingCarriageReturns)
{
  littlebot_base::SerialPort port;
  std::vector<uint8_t> payload;

  port.injectRxData("\r\r\r[OK]\n");

  ASSERT_GT(port.read(payload), 0);
  EXPECT_EQ(payload, toBytes("OK"));
}

TEST(SerialPortTest, NoFrameDoesNotThrow)
{
  littlebot_base::SerialPort port;
  std::vector<uint8_t> payload;

  port.injectRxData("random noise");

  EXPECT_NO_THROW({
    EXPECT_EQ(port.read(payload), 0);
  });
}
