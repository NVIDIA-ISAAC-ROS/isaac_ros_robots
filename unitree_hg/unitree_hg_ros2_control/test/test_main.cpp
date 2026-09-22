// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

// The Unitree SDK archive contains service_main.cpp.o with its own main(). If selected, it starts
// the standalone SDK service, looks for _test_*_noshim.json, and segfaults before any tests run
// when that service configuration is absent. Defining main here ensures GoogleTest starts instead.
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
