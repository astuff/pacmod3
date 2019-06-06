/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <pacmod3/pacmod3_core.h>
#include <gtest/gtest.h>

#include <vector>
#include <memory>
#include <unordered_map>

using namespace AS::Drivers::PACMod3;  // NOLINT

TEST(PACMod3Core, generateRptMessages)
{
  std::vector<uint32_t> rpt_ids =
  {
     0x10,  0x20,
    0x200, 0x204, 0x208, 0x20C, 0x210, 0x214, 0x218, 0x21C,
    0x220, 0x224, 0x228, 0x22C, 0x230, 0x234, 0x300, 0x304,
    0x318, 0x328, 0x32C, 0x330, 0x334, 0x400, 0x401, 0x402,
    0x403, 0x404, 0x405, 0x406, 0x407, 0x408, 0x409, 0x40A,
    0x40D, 0x40E, 0x40F, 0x410, 0x411, 0x412, 0x413, 0x414,
    0x415, 0x416, 0x417, 0x418
  };

  std::unordered_map<uint32_t, std::shared_ptr<Pacmod3TxMsg>> generated_msgs;
  std::shared_ptr<Pacmod3TxMsg> invalid_ptr(nullptr);
  generated_msgs.reserve(0x800);

  for (uint32_t i = 0; i < 0x800; i++)
  {
    auto gen_msg = Pacmod3TxMsg::make_rpt_message(i);
    generated_msgs[i] = gen_msg;
  }

  for (const auto& msg : generated_msgs)
  {
    auto id_valid = std::find(rpt_ids.begin(), rpt_ids.end(), msg.first);

    if (id_valid != rpt_ids.end())
    {
      // ID is a valid one so make_rpt_message
      // should return a valid shared_ptr
      ASSERT_TRUE(msg.second) << "Valid CAN ID 0x"
        << std::hex << msg.first << " did not return a valid message type.";
    }
    else
    {
      // ID is not a valid one so make_rpt_message
      // should return a shared_ptr to nullptr
      ASSERT_FALSE(msg.second) << "Invalid CAN ID 0x"
        << std::hex << msg.first << " generated a valid message type.";
    }
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
