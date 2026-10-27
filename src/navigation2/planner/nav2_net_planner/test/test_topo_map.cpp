#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "nav2_net_planner/topo_map.hpp"

namespace
{

constexpr double kOriginLatitude = 30.81409204;
constexpr double kOriginLongitude = 120.83329261;
constexpr double kOriginHeight = 12.543300;

TEST(TopoMapCoordinates, OriginMapsToZero)
{
  double east = 1.0, north = 1.0, up = 1.0;
  TopoMap::llaToEnu(
    kOriginLatitude, kOriginLongitude, kOriginHeight,
    kOriginLatitude, kOriginLongitude, kOriginHeight, &east, &north, &up);
  EXPECT_NEAR(east, 0.0, 1.0e-9);
  EXPECT_NEAR(north, 0.0, 1.0e-9);
  EXPECT_NEAR(up, 0.0, 1.0e-9);
}

TEST(TopoMapCoordinates, MatchesWgs84Reference)
{
  double east = 0.0, north = 0.0, up = 0.0;
  TopoMap::llaToEnu(
    30.82309204, 120.84329261, 20.0,
    kOriginLatitude, kOriginLongitude, kOriginHeight, &east, &north, &up);
  EXPECT_NEAR(east, 956.804155898, 1.0e-6);
  EXPECT_NEAR(north, 997.843074947, 1.0e-6);
  EXPECT_NEAR(up, 7.306622946, 1.0e-6);
}

TEST(TopoMapCoordinates, EnuRoundTripPreservesLla)
{
  constexpr double latitude = 30.82309204;
  constexpr double longitude = 120.84329261;
  constexpr double height = 20.0;
  double east = 0.0, north = 0.0, up = 0.0;
  TopoMap::llaToEnu(
    latitude, longitude, height,
    kOriginLatitude, kOriginLongitude, kOriginHeight, &east, &north, &up);
  double recoveredLatitude = 0.0, recoveredLongitude = 0.0, recoveredHeight = 0.0;
  TopoMap::enuToLla(
    east, north, up, kOriginLatitude, kOriginLongitude, kOriginHeight,
    &recoveredLatitude, &recoveredLongitude, &recoveredHeight);
  EXPECT_NEAR(recoveredLatitude, latitude, 1.0e-10);
  EXPECT_NEAR(recoveredLongitude, longitude, 1.0e-10);
  EXPECT_NEAR(recoveredHeight, height, 1.0e-5);
}

TEST(TopoMapCoordinates, LoadingLlaMapReprojectsStoredXyzAndEdgeCost)
{
  const std::string text = R"(
map:
1 road
lla_mode:
1
origin:
30.81409204 120.83329261 12.543300
node:
1 0 1 999 999 999 30.81409204 120.83329261 12.543300
2 0 1 999 999 999 30.82309204 120.84329261 20.0
edge:
1 0 1 1 2 0
)";
  TopoMap map;
  ASSERT_TRUE(map.loadTopoMapFromString(text));
  ASSERT_EQ(map.nodes.size(), 2U);
  ASSERT_EQ(map.edges.size(), 1U);
  EXPECT_NEAR(map.nodes[0].x, 0.0, 1.0e-3);
  EXPECT_NEAR(map.nodes[1].x, 956.804155898, 1.0e-3);
  EXPECT_NEAR(map.nodes[1].y, 997.843074947, 1.0e-3);
  EXPECT_NEAR(map.nodes[1].z, 7.306622946, 1.0e-3);
  EXPECT_NEAR(map.edges[0].cost,
    std::sqrt(956.804155898 * 956.804155898 +
    997.843074947 * 997.843074947 + 7.306622946 * 7.306622946), 1.0e-3);
}

TEST(TopoMapCoordinates, LoadingPlanarMapKeepsStoredXyz)
{
  const std::string text = R"(
map:
1 road
lla_mode:
0
origin:
30.81409204 120.83329261 12.543300
node:
1 0 1 10 20 30 30.82309204 120.84329261 20.0
edge:
)";
  TopoMap map;
  ASSERT_TRUE(map.loadTopoMapFromString(text));
  ASSERT_EQ(map.nodes.size(), 1U);
  EXPECT_FLOAT_EQ(map.nodes.front().x, 10.0F);
  EXPECT_FLOAT_EQ(map.nodes.front().y, 20.0F);
  EXPECT_FLOAT_EQ(map.nodes.front().z, 30.0F);
}

}  // namespace
