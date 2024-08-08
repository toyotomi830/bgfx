#include "SphereSegmentation.h"
#include <algorithm>
#include <iterator>
#include <set>

namespace segment_mgr
{
	SphereSegmentMgr::SphereSegmentMgr(float radius, int Size, float cellS)
		: radius(radius)
	{
		tileSize = Size;
		cellSize = cellS;
		unitRadianSize_ = float(M_PI) / (float(M_PI) * radius / (cellSize * tileSize));  // 计算每个Tile的弧度

		glm::mat4 m(1.0f);
		m = glm::scale(glm::mat4(1.0f), glm::vec3(radius, radius, radius));
		int N = int(float(M_PI) / unitRadianSize_ + 2.0f);
		tiles.resize(N + 1);
		float dl = 180.0f / N;
		for (int i = 0; i <= N; ++i) {
			float degree = -90.0f + i * dl;
			float rad = degree / 360.0f * float(M_PI) * 2.0f;
			float y = sinf(rad);
			float r = cosf(rad);
			float perimeter = r * float(M_PI) * 2;
			if (perimeter <= unitRadianSize_) {
				tiles[i].resize(1);
				Tile& t = tiles[i][0];
				if (degree > 0) {
					t.centerPos = { 0, 1, 0 };
					t.axis_u = { 1, 0, 0 };
					t.axis_v = { 0, 0, -1 };
				}
				else {
					t.centerPos = { 0, -1, 0 };
					t.axis_u = { 1, 0, 0 };
					t.axis_v = { 0, 0, 1 };
				}
				t.centerPos = m * glm::vec4(t.centerPos, 1.0f);
				t.tileIndex = totalTilesNum;
				totalTilesNum++;
			}
			else {
				int NUM = int(perimeter / unitRadianSize_ + 4.0f);
				tiles[i].resize(NUM);
				float da = float(M_PI) * 2.0f / NUM;
				for (int j = 0; j < NUM; ++j) {
					float lat = da * j;
					float x = r * cosf(lat);
					float z = r * sinf(lat);
					Tile& t = tiles[i][j];
					t.centerPos = { x, y, z };
					t.axis_u = glm::normalize(glm::cross(t.centerPos, glm::vec3(0, 1, 0)));
					t.axis_v = glm::normalize(glm::cross(t.centerPos, t.axis_u));
					t.centerPos = m * glm::vec4(t.centerPos, 1.0f);
					t.tileIndex = totalTilesNum;
					totalTilesNum++;
				}
			}
		}
		listData.reserve(totalTilesNum * tileSize * tileSize);
		handleData.reserve(totalTilesNum * tileSize * tileSize);
		for (auto&& i : tiles)
		{
			for (auto&& j : i)
			{
				glm::vec3 centerPoint = j.centerPos;
				glm::vec3 u = j.axis_u;
				glm::vec3 v = j.axis_v;
				glm::vec3 negu = -u;
				glm::vec3 negv = -v;
				glm::vec3 toMin = (negu * cellSize * float(tileSize) / 2.0f) + (negv * cellSize * float(tileSize) / 2.0f);
				glm::vec3 MinP = centerPoint + toMin;
				for (int x = 0; x < tileSize; x++)
				{
					for (int z = 0; z < tileSize; z++)
					{
						glm::vec3 ListMinPoint = MinP + (u * float(x) * cellSize) + (v * float(z) * cellSize);
						glm::vec3 ListCenterPoint = ListMinPoint + (u * cellSize / 2.0f) + v * float(cellSize / 2.0f);
						listData.emplace_back(ListCenterPoint);
						handleData.emplace_back(j.tileIndex, x, z);
					}
				}
			}
		}
		for (auto&& i : tiles)
		{
			for (auto&& j : i)
			{
				glm::vec3 centerPoint = j.centerPos;
				glm::vec3 u = j.axis_u;
				glm::vec3 v = j.axis_v;
				glm::vec3 negu = -u;
				glm::vec3 negv = -v;
				glm::vec3 toMin = (negu * cellSize * float(tileSize) / 2.0f) + (negv * cellSize * float(tileSize) / 2.0f);
				glm::vec3 MinP = centerPoint + toMin;
				for (int x = 0; x < tileSize; x++)
				{
					for (int z = 0; z < tileSize; z++)
					{
						int ListIndexNow = j.tileIndex * tileSize * tileSize + x * tileSize + z;
						auto&& List = listData[ListIndexNow];
						if (x != 0)
						{
							List.neighborsIndex[0] = ListIndexNow - 1;
						}
						else
						{
							List.neighborsIndex[0] = (OffsetGetEdgeSpanListNeighborIndex(List, EdgeNeighborDirect::kNegX));
							listData[List.neighborsIndex[0]].neighborsIndex[2] = ListIndexNow;
						}
						if (x != tileSize - 1)
						{
							List.neighborsIndex[2] = (ListIndexNow + 1);
						}
						else
						{
							List.neighborsIndex[2] = (OffsetGetEdgeSpanListNeighborIndex(List, EdgeNeighborDirect::kX));
						}
						if (z != 0)
						{
							List.neighborsIndex[1] = (ListIndexNow - tileSize);
						}
						else
						{
							List.neighborsIndex[1] = (OffsetGetEdgeSpanListNeighborIndex(List, EdgeNeighborDirect::kNegZ));
						}
						if (z != tileSize - 1)
						{
							List.neighborsIndex[3] = (ListIndexNow + tileSize);
						}
						else
						{
							List.neighborsIndex[3] = (OffsetGetEdgeSpanListNeighborIndex(List, EdgeNeighborDirect::kZ));
							listData[List.neighborsIndex[3]].neighborsIndex[1] = ListIndexNow;
						}
					}
				}
			}
		}
	}

	int SphereSegmentMgr::GetTileIndexFromWorldPos(const glm::vec3& xyz, const glm::vec3& center, const glm::vec3& vecDir) const
	{
		auto&& [longitudeIndex, patchIndex] = Get2TileIndexFromWorldPos(xyz.x - center.x, xyz.y - center.y, xyz.z - center.z);
		return tiles[longitudeIndex][patchIndex].tileIndex;
	}

	const CellPos3D SphereSegmentMgr::GetCellId3DFromWorldPos(const glm::vec3& xyz, const glm::vec3& center, const HeightParams& parm, const glm::vec3& vecDir) const
	{
		auto&& temp = xyz - center;
		auto id = handleData[GetSpanListIndexFromWorldPos(temp.x, temp.y, temp.z)];
		float length = glm::length(temp);

		float div = length - (this->radius + parm.bottom);
		uint16_t y = (uint16_t)(div / parm.cellHeight);

		auto&& pos = listData[CellIdGetIndice(id)].centerPos;//why
		return CellPos3D(id.tileId, id.x, y, id.z);
	}

	std::tuple<float, float> SphereSegmentMgr::GetSpanHeight(unsigned int smin, unsigned smax, const HeightParams& parm) const
	{
		float bottomHeight = smin * parm.cellHeight + parm.bottom + radius;
		float topHeight = smax * parm.cellHeight + parm.bottom + radius;

		return std::make_tuple(bottomHeight, topHeight);
	}

	glm::vec3 SphereSegmentMgr::GetWorldPosFromCellId3D(const CellPos3D& Id, const glm::vec3& Center, const HeightParams& Parm, const glm::vec3& vecDir) const
	{
		glm::vec3 glmCenter(Center.x, Center.y, Center.z);
		CellId cId = static_cast<CellId>(Id);
		glm::vec3 cellPos = listData[CellIdGetIndice(cId)].centerPos;
		cellPos = cellPos + glmCenter;

		glm::vec3 dirVec = glm::normalize(cellPos - glmCenter);
		float Length = (Parm.cellHeight * (float(Id.y) + 0.5f));
		float Height = (Parm.bottom + Length);

		glm::vec3 toFinPosVec = dirVec * Height;

		glm::vec3 finalPos = cellPos + toFinPosVec;
		return glm::vec3(finalPos.x, finalPos.y, finalPos.z);
	}

	float SphereSegmentMgr::GetCellId3dDistance(const CellPos3D& s1, const CellPos3D& s2, const HeightParams& parm, DistanceFuncEnum f) const
	{
		float dis = 0.0f;
		const Tile& t1 = GetTileByIndex(s1.tileId);
		const Tile& t2 = GetTileByIndex(s2.tileId);

		const Cell& c1 = listData[CellIdGetIndice(s1)];
		const Cell& c2 = listData[CellIdGetIndice(s2)];

		switch (f)
		{
		case kEuclidean:
		{
			glm::vec3 y1 = glm::normalize(glm::cross(t1.axis_u, t1.axis_v));
			glm::vec3 y2 = glm::normalize(glm::cross(t2.axis_u, t2.axis_v));
			glm::vec3 p1 = c1.centerPos + y1 * (parm.bottom + (parm.cellHeight * (s1.y + 0.5f)));
			glm::vec3 p2 = c2.centerPos + y2 * (parm.bottom + (parm.cellHeight * (s2.y + 0.5f)));
			dis = glm::distance(p1, p2);
			break;
		}
		case kSpherical:
		{
			float cos = glm::dot(glm::normalize(c1.centerPos), glm::normalize(c2.centerPos));
			dis = radius * acos(cos);
			break;
		}
		case kSphericalManhattan://why do not use cell id&tile to calculate or using longgitude and latitude
		{
			glm::vec3 p1 = c1.centerPos;
			glm::vec3 p2 = c2.centerPos;
			float radians1 = asin(p1.y / radius);
			float radians2 = asin(p2.y / radius);

			float radiansSum1 = 0;
			if (p1.y * p2.y < 0)
				radiansSum1 = radians1 + radians2;
			else
				radiansSum1 = std::abs(radians1 - radians2);

			if (radiansSum1 > M_PI)
				radiansSum1 = 2 * M_PI - radiansSum1;

			float dis1 = radius * radiansSum1;

			float radius2 = std::sqrt((radius * radius) - (p1.y * p1.y));

			float radians3 = atan(p1.z / p1.x);
			float radians4 = atan(p2.z / p2.x);
			float radiansSum2 = 0;
			if (p1.z * p2.z < 0)
				radiansSum2 = radians3 + radians4;
			else
				radiansSum2 = std::abs(radians3 - radians4);

			if (radiansSum2 > M_PI)
				radiansSum2 = 2 * M_PI - radiansSum2;

			float dis2 = radius2 * radiansSum2;

			float disHeight = std::abs((parm.bottom + (parm.cellHeight * (s1.y + 0.5f))) - (parm.bottom + (parm.cellHeight * (s2.y + 0.5f))));

			dis = dis1 + dis2 + disHeight;

			break;
		}
		}
		return dis;
	}

	std::vector<int> SphereSegmentMgr::GetSquareTileIds(const int TileId1, const int TileId2) const//find minimal ract tile set contain 2 tiles
	{
		std::vector<int> result;
		result.emplace_back(TileId1);
		result.emplace_back(TileId2);

		auto&& [longitude1, Horizontal1] = Get2TileIndexFrom1TileIndex(TileId1);
		auto&& [longitude2, Horizontal2] = Get2TileIndexFrom1TileIndex(TileId2);
		float minRadians = (float)std::min(2.0f * M_PI * (float)Horizontal1 / (float)tiles[longitude1].size(), 2.0f * M_PI * (float)Horizontal2 / (float)tiles[longitude2].size());
		float maxRadians = (float)std::max(2.0f * M_PI * (float)Horizontal1 / (float)tiles[longitude1].size(), 2.0f * M_PI * (float)Horizontal2 / (float)tiles[longitude2].size());

		bool reverse = false;
		float durationRadians = maxRadians - maxRadians;

		if (durationRadians > M_PI)
			reverse = true;

		int minLatitudeIndex = std::min(longitude1, longitude2);
		int maxLatitudeIndex = std::max(longitude1, longitude2);

		auto vectorBool = [](const std::vector<Tile>& v1, std::vector<int>& v2) -> std::vector<int>//filter v1 element  caontain by v2
		{
			std::vector<int> result;
			for (auto&& i : v1)
			{
				if (std::find(v2.begin(), v2.end(), i.tileIndex) == v2.end())
				{
					result.emplace_back(i.tileIndex);
				}
			}
			return result;
		};

		for (int i = minLatitudeIndex; i <= maxLatitudeIndex; i++)
		{
			int index;
			for (index = 0; index < (int)tiles[i].size(); index++)
			{
				if (2.0f * M_PI * index / (float)tiles[i].size() >= minRadians)//meaning for this?
				{
					index--;
					index = std::clamp(index, 0, (int)tiles[i].size() - 1);
					break;
				}
			}
			float unionRadians = 2.0f * M_PI / (float)tiles[i].size();
			int size = int(durationRadians / unionRadians) + 1;
			std::vector<int> tempResult;
			for (int j = 0; j <= size; j++)
			{
				int h = index + j;
				if (h >= tiles[i].size())
					h -= tiles[i].size();
				tempResult.emplace_back(tiles[i][h].tileIndex);
			}
			if (reverse) tempResult = vectorBool(tiles[i], tempResult);

			result.insert(result.end(), tempResult.begin(), tempResult.end());
		}
		return result;
	}

	std::vector<int> SphereSegmentMgr::GetTriangleTileIds(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& center, const glm::vec3& vecDir) const //unused?
	{
		std::vector<int> result;
		int Tile1 = GetTileIndexFromWorldPos(p1, center);
		int Tile2 = GetTileIndexFromWorldPos(p2, center);
		int Tile3 = GetTileIndexFromWorldPos(p3, center);
		if (Tile1 == Tile2 == Tile3)
		{
			return std::vector<int>(Tile1);
		}
		std::vector<int> v1(std::move(GetSquareTileIds(Tile1, Tile2)));
		std::vector<int> v2(std::move(GetSquareTileIds(Tile2, Tile3)));
		std::vector<int> v3(std::move(GetSquareTileIds(Tile3, Tile1)));

		std::set<int> resultSet;
		std::copy(v1.begin(), v1.end(), std::inserter(resultSet, resultSet.begin()));
		std::copy(v2.begin(), v2.end(), std::inserter(resultSet, resultSet.begin()));
		std::copy(v3.begin(), v3.end(), std::inserter(resultSet, resultSet.begin()));

		std::copy(resultSet.begin(), resultSet.end(), result.begin());
		return result;
	}

	int SphereSegmentMgr::GetSpanListIndexFromWorldPos(float x, float y, float z) const//return cell id from tile index and cartesian coordinates;
	{
		auto&& [latitudeIndex, longitudeIndex] = Get2TileIndexFromWorldPos(x, y, z);
		const Tile& tile = tiles[latitudeIndex][longitudeIndex];
		glm::vec3 axis_y = glm::normalize(glm::cross(tile.axis_u, tile.axis_v));
		glm::mat4 matrix = glm::mat4(glm::vec4(tile.axis_u, 0), glm::vec4(axis_y, 0), glm::vec4(tile.axis_v, 0), glm::vec4(0, 0, 0, 1));
		matrix = glm::transpose(matrix);

		glm::vec3 worldPos(x, y, z);
		worldPos = worldPos;
		glm::vec3 Porj2SphereWolrdPos = glm::normalize(worldPos) * radius;

		glm::vec3 SpanListPos = matrix * glm::vec4(Porj2SphereWolrdPos, 1.0f);

		glm::vec3 centerPoint = tile.centerPos;
		glm::vec3 u = tile.axis_u;
		glm::vec3 v = tile.axis_v;
		glm::vec3 negu = -u;
		glm::vec3 negv = -v;
		glm::vec3 toMin = (negu * cellSize * float(tileSize) / 2.0f) + (negv * cellSize * float(tileSize) / 2.0f);
		glm::vec3 MinP = centerPoint + toMin;
		glm::vec3 temp = (MinP - tile.centerPos);
		MinP = matrix * glm::vec4(temp, 1.0f);

		int xIndex = (int)((SpanListPos.x - MinP.x) / cellSize);
		int zIndex = (int)((SpanListPos.z - MinP.z) / cellSize);
		xIndex = std::clamp(xIndex, 0, tileSize - 1);

		zIndex = std::clamp(zIndex, 0, tileSize - 1);
		return tile.tileIndex * tileSize * tileSize + xIndex * tileSize + zIndex;
	}

	int SphereSegmentMgr::OffsetGetEdgeSpanListNeighborIndex(const Cell& sl, EdgeNeighborDirect e) {
		glm::vec3 Center = sl.centerPos;
		glm::mat4 rotate(1.0f);
		if (e == EdgeNeighborDirect::kNegX)
		{
			int latitude = GetLatitudeIndex(Center.x, Center.y, Center.z);
			float HorizontalRadiansStride = 360.0f / float(tiles[latitude].size() * tileSize) / 180 * float(M_PI);
			rotate = glm::rotate(rotate, HorizontalRadiansStride, glm::vec3(0, 1, 0));
			glm::vec3 pos = rotate * glm::vec4(Center, 1.0f);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}

		if (e == EdgeNeighborDirect::kX) {
			int latitude = GetLatitudeIndex(Center.x, Center.y, Center.z);
			float HorizontalRadiansStride = 360.0f / float(tiles[latitude].size() * tileSize) / 180 * float(M_PI);
			rotate = glm::rotate(rotate, HorizontalRadiansStride, glm::vec3(0, -1, 0));
			glm::vec3 pos = rotate * glm::vec4(Center, 1.0f);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}

		if (e == EdgeNeighborDirect::kNegZ) {
			float LatitudeAngleStride = float(M_PI) / (float(tiles.size() - 1) * float(tileSize));
			glm::vec3 rotateAxis = glm::normalize(glm::cross((Center), glm::vec3(0, 1, 0)));
			rotate = glm::rotate(rotate, LatitudeAngleStride, rotateAxis);
			glm::vec3 pos = rotate * glm::vec4(Center, 1.0f);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}

		if (e == EdgeNeighborDirect::kZ) {
			float LatitudeAngleStride = float(M_PI) / (float(tiles.size() - 1) * float(tileSize));
			glm::vec3 rotateAxis = glm::normalize(glm::cross((Center), glm::vec3(0, -1, 0)));
			rotate = glm::rotate(rotate, LatitudeAngleStride, rotateAxis);
			glm::vec3 pos = rotate * glm::vec4(Center, 1.0f);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}
		return 0;
	}

	std::tuple<int, int> SphereSegmentMgr::Get2TileIndexFromWorldPos(float x, float y, float z)const
	{
		glm::vec3 PositionVector = { x, y, z };
		int latitudeIndex = GetLatitudeIndex(x, y, z);
		if (latitudeIndex == 0 || latitudeIndex == (int)(tiles.size() - 1)) {
			return std::make_tuple(latitudeIndex, 0);
		}
		int longitudeIndex = 0;
		float dot1 = glm::dot(glm::normalize(PositionVector), glm::vec3(0, 1, 0));
		float dot2 = glm::dot(glm::normalize(glm::vec3(PositionVector.x, 0, PositionVector.z)), glm::vec3(1, 0, 0));
		float radians = (float)(PositionVector.z >= 0 ? acos(dot2) : 2 * M_PI - acos(dot2));
		float HorizontalStride = 2.0f * float(M_PI) / float(tiles[latitudeIndex].size());
		float radius2 = std::abs(radius * sin(acos(dot1)));//get the radius for the section circle
		float stride = atan((GetCellSize() * (float)tileSize / 2.0f) / radius2);

		if ((2 * M_PI - radians) < stride / 2.0)
		{
			return std::make_tuple(latitudeIndex, 0);
		}
		else
		{
			int tempIndex = (int)(radians / HorizontalStride);
			float right = float(tempIndex + 1) * HorizontalStride;
			longitudeIndex = (right - radians) < stride ? tempIndex + 1 : tempIndex;
		}
		if (longitudeIndex == (int)tiles[latitudeIndex].size())
		{
			longitudeIndex = 0;
		}
		return std::make_tuple(latitudeIndex, longitudeIndex);
	}

	int SphereSegmentMgr::GetLatitudeIndex(float x, float y, float z) const
	{
		glm::vec3 PositionVector = { x, y, z };

		float LatitudeAngleStride = M_PI / float(tiles.size() - 1);
		float dot = glm::dot(glm::normalize(PositionVector), glm::vec3(0, -1, 0));
		float angle = acos(dot);

		int latitudeIndex = 0;
		int patchIndex = 0;

		if ((M_PI - angle) < atan((GetCellSize() * (float)tileSize / 2.0f) / radius))//check if point at top tile
		{
			latitudeIndex = (int)(tiles.size() - 1);
		}
		else
		{
			int tempIndex = (int)(angle / LatitudeAngleStride);

			float right = LatitudeAngleStride * float(tempIndex + 1);

			latitudeIndex = (right - angle) < atan((GetCellSize() * (float)tileSize / 2.0f) / radius) ? tempIndex + 1 : tempIndex;//rounding latitude
		}
		return latitudeIndex;
	}


}


