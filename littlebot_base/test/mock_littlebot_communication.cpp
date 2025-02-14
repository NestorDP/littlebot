#include <gmock/gmock.h>

#include "littlebot_base/littlebot_communication_interface.hpp"

class MockLittlebotCommunication : public MockLittlebotCommunicationInterface
{
public:
  MOCK_METHOD(int, GetData, (int id), (const, override));
};