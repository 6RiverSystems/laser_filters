/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <filters/filter_chain.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <pluginlib/class_loader.h>

#define GTEST_COUT std::cerr

sensor_msgs::LaserScan gen_msg(){
  sensor_msgs::LaserScan msg;

  float temp[] = {1.0, 0.1, 1.0, 1.0, 1.0, 9.0, 1.0, 1.0, 1.0, 2.3};
  std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "laser";
  msg.angle_min = -.5;
  msg.angle_max = .5;
  msg.angle_increment = 0.1;
  msg.time_increment = 0.1;
  msg.scan_time = 0.1;
  msg.range_min = 0.5;
  msg.range_max = 1.5;
  msg.ranges = v1;
  msg.intensities = v1;

  return msg;
}

/** Verifies that two vectors of range values are the same. Allows the case
 * where corresponding values are both NaN.
 */
void expect_ranges_eq(const std::vector<float> &a, const std::vector<float> &b) {
  for( int i=0; i<10; i++) {
    if(std::isnan(a[i])) {
      EXPECT_TRUE(std::isnan(b[i]));
    }
    else {
      EXPECT_NEAR(a[i], b[i], 1e-6);
    }
  }
}

void print_ranges(const std::vector<float> &a) {
  for(auto i = 0; i < a.size(); i++)
  {
    // Zero range usually means obstacle is further than LIDAR max range
    // Zero intensity occurs when the ray reflects from reflective object or another LIDAR
    // NAN values present when points rejected by a filter
    GTEST_COUT << a[i] << " ";
  }
  GTEST_COUT << "\n";
}

TEST(ScanToScanFilterChain, BadConfiguration)
{
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");

  try
  {
    filter_chain_.configure("bad_filter_chain");
  }
  catch(pluginlib::LibraryLoadException)
  {
    EXPECT_FALSE(false);
  }
  
  filter_chain_.clear();
}

TEST(ScanToScanFilterChain, IntensityFilter)
{
  sensor_msgs::LaserScan msg_in, msg_out, expected_msg;
  float nanval = std::numeric_limits<float>::quiet_NaN();
  float temp[] = {1.0, nanval, 1.0, 1.0, 1.0, nanval, 1.0, 1.0, 1.0, 2.3};
  std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));
  expected_msg.ranges = v1;
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");

  EXPECT_TRUE(filter_chain_.configure("intensity_filter_chain"));

  msg_in = gen_msg();

  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  expect_ranges_eq(msg_out.ranges, expected_msg.ranges);

  filter_chain_.clear();
}

TEST(ScanToScanFilterChain, InterpFilter)
{
  sensor_msgs::LaserScan msg_in, msg_out, expected_msg;
  float temp[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));
  expected_msg.ranges = v1;
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");

  EXPECT_TRUE(filter_chain_.configure("interp_filter_chain"));

  msg_in = gen_msg();

  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  
  for( int i=0; i<10; i++){
  EXPECT_NEAR(msg_out.ranges[i],expected_msg.ranges[i],1e-6);
  }

  filter_chain_.clear();
}

TEST(ScanToScanFilterChain, ShadowFilter)
{
  sensor_msgs::LaserScan msg_in, msg_out, expected_msg;
  float nanval = std::numeric_limits<float>::quiet_NaN();
  float temp[] = {nanval, 0.1, nanval, 1.0, 1.0, nanval, 1.0, 1.0, 1.0, nanval};
  std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));
  expected_msg.ranges = v1; 
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");

  EXPECT_TRUE(filter_chain_.configure("shadow_filter_chain"));

  msg_in = gen_msg();

  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));

  expect_ranges_eq(msg_out.ranges, expected_msg.ranges);

  filter_chain_.clear();
}

TEST(ScanToScanFilterChain, ArrayFilter)
{
  sensor_msgs::LaserScan msg_in, msg_out, expected_msg;
  float temp[] = {1.0, 0.4, 1.0, 1.0, 1.0, 6.3333, 1.0, 1.0, 1.0, 1.8667};
  std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));
  expected_msg.ranges = v1; 
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");

  EXPECT_TRUE(filter_chain_.configure("array_filter_chain"));

  msg_in = gen_msg();
  
  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  float temp2[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<float> v2 (temp2, temp2 + sizeof(temp2) / sizeof(float));
  msg_in.ranges = v2;
  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  msg_in = gen_msg();
  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  
  for( int i=0; i<10; i++){
    EXPECT_NEAR(msg_out.ranges[i],expected_msg.ranges[i],1e-3);
    EXPECT_NEAR(msg_out.intensities[i],msg_in.intensities[i],1e-3);
  }
  filter_chain_.clear();
}

TEST(ScanToScanFilterChain, RadiusFilter)
{
  sensor_msgs::LaserScan msg_in, msg_out, expected_msg;
  // test basic functionality
  float nanval = std::numeric_limits<float>::quiet_NaN();
  float temp[] = {nanval, nanval, 1.0, 1.0, 1.0, nanval, 1.0, 1.0, 1.0, nanval};
  std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));
  expected_msg.ranges = v1;
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");

  EXPECT_TRUE(filter_chain_.configure("radius_filter_chain"));

  msg_in = gen_msg();

  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  expect_ranges_eq(msg_out.ranges, expected_msg.ranges);

  // test cur_threshold is setup correctly and we consider euclidean distance rather than range only
  float temp2[] = {0.9, 0.8, 1.0, 1.0, 1.1, 9.0, 1.0, 1.0, 1.0, 2.3};
  std::vector<float> v2 (temp2, temp2 + sizeof(temp2) / sizeof(float));
  msg_in.ranges = v2;
  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));

  float temp3[] = {nanval, nanval, nanval, 1.0, nanval, nanval, 1.0, 1.0, 1.0, nanval};
  std::vector<float> v3 (temp3, temp3 + sizeof(temp3) / sizeof(float));
  expected_msg.ranges = v3;
  expect_ranges_eq(msg_out.ranges, expected_msg.ranges);

  // test cur_threshold is adaptive to the range of the point
  float temp4[] = {4.9, 5.39, 4.41, 1.0, 8.0, 8.89, 7.11, 1.0, 1.0, 0.9};
  std::vector<float> v4 (temp4, temp4 + sizeof(temp4) / sizeof(float));
  msg_in.ranges = v4;
  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));

  float temp5[] = {nanval, nanval, nanval, nanval, nanval, nanval, nanval, nanval, 1.0, nanval};
  std::vector<float> v5 (temp5, temp5 + sizeof(temp5) / sizeof(float));
  expected_msg.ranges = v5;
  expect_ranges_eq(msg_out.ranges, expected_msg.ranges);

  filter_chain_.clear();
}

TEST(ScanToScanFilterChain, Validator)
{
  sensor_msgs::LaserScan msg_in, msg_out, expected_msg;
  // test basic functionality
  float nanval = std::numeric_limits<float>::quiet_NaN();
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");

  EXPECT_TRUE(filter_chain_.configure("validator_filter_chain"));

  msg_in = gen_msg();

  //Test: passthrough
  float temp[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<float> v (temp, temp + sizeof(temp) / sizeof(float));
  expected_msg.ranges = v;

  float temp1[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<float> v1 (temp1, temp1 + sizeof(temp1) / sizeof(float));
  msg_in.ranges = v1;

  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  expect_ranges_eq(msg_out.ranges, expected_msg.ranges);

  //Test: occlustion threshold. Many low values inside contour
  float temp3[] = {0.4, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 1.0};
  std::vector<float> v3 (temp3, temp3 + sizeof(temp3) / sizeof(float));
  msg_in.ranges = v3;

  EXPECT_FALSE(filter_chain_.update(msg_in, msg_out)) << "should fail due to occlusion threshold";

  //Test: trigger invalid threshold
  float temp4[] = {nanval, nanval, nanval, nanval, nanval, nanval, nanval, nanval, 1.0, 1.0};
  std::vector<float> v4 (temp4, temp4 + sizeof(temp4) / sizeof(float));
  expected_msg.ranges = v4;

  float temp5[] = {nanval, nanval, nanval, nanval, nanval, nanval, nanval , nanval, 1.0, 1.0};
  std::vector<float> v5 (temp5, temp5 + sizeof(temp5) / sizeof(float));
  msg_in.ranges = v5;

  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  expect_ranges_eq(msg_out.ranges, expected_msg.ranges);
  //TODO: expect log warning for invalid points

  filter_chain_.clear();
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_scan_to_scan_filter_chain");
  return RUN_ALL_TESTS();
}

