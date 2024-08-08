#include "CylinderSegmentation.h"

namespace segment_mgr
{
	CylinderSegmentMgr::CylinderSegmentMgr(float _radius, float _height, int _tileSize, float _cellSize)
		:radius(_radius), height(_height)
	{
		tileSize = _tileSize;
		cellSize = _cellSize;
		float unitRadians = (float)(2.0f * asin((float(tileSize) * cellSize) / 2.0f / radius));
		int RingSize, verticalSize;
		RingSize = (int)(2.0f * M_PI / unitRadians) + 1;
		unitRadians = 2.0f * M_PI / RingSize;
		cellSize = radius * sin(unitRadians / 2.0f) * 2.0f / float(tileSize);
		verticalSize = int(height / (float(tileSize) * cellSize)) + 1;
		float bottom_y = -height / 2.0f;
		this->tiles.resize(verticalSize);
		for (int i = 0; i < verticalSize; i++)
		{
			float y_current = bottom_y + ((i + 0.5f) * tileSize * cellSize);
			float radius2 = radius * cos(unitRadians / 2.0f);
			auto&& ts = tiles[i];
			for (int j = 0; j < RingSize; j++)
			{
				Tile t;
				float radians = j * unitRadians;
				float x = radius2 * cos(radians);
				float z = radius2 * sin(radians);
				t.centerPos = glm::vec3(x, y_current, z);
				glm::vec3 axis_y(x, 0, z);
				t.axis_v = glm::vec3(0, 1, 0);
				t.axis_u = glm::normalize(glm::cross(axis_y, t.axis_v));
				t.tileIndex = totalTilesNum;
				totalTilesNum++;
				ts.emplace_back(std::move(t));
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
	//修改完毕
	int CylinderSegmentMgr::GetTileIndexFromWorldPos(const glm::vec3& xyz, const glm::vec3& center, const glm::vec3& vecDir) const
	{
		auto&& m = ToWorldPosMat(vecDir);
		glm::vec4 relativePos(xyz.x - center.x, xyz.y - center.y, xyz.z - center.z, 1);
		glm::vec4 Pos = m * relativePos;
		auto&& [longitudeIndex, patchIndex] = Get2TileIndexFromWorldPos(Pos.x, Pos.y, Pos.z);
		return tiles[longitudeIndex][patchIndex].tileIndex;
	}

	//修改完毕
	const CellPos3D CylinderSegmentMgr::GetCellId3DFromWorldPos(const glm::vec3& xyz, const glm::vec3& center, const HeightParams& parm,
		const glm::vec3& vecDir) const
	{
		auto&& m = ToWorldPosMat(vecDir);
		glm::vec4 relativePos(xyz.x - center.x, xyz.y - center.y, xyz.z - center.z, 1);
		auto&& temp = m * relativePos;

		auto id = handleData[GetSpanListIndexFromWorldPos(temp.x, temp.y, temp.z)];
		float length = glm::length(glm::vec3(temp.x, 0, temp.z));

		float div = (this->radius - parm.bottom) - length;
		uint16_t y = (uint16_t)(div / parm.cellHeight);

		auto&& pos = listData[CellIdGetIndice(id)].centerPos;
		return CellPos3D(id.tileId, id.x, y, id.z);
	}
	// 修改完毕,圆柱体和别的不一样，这里可能要改
	std::tuple<float, float> CylinderSegmentMgr::GetSpanHeight(unsigned int smin, unsigned smax, const HeightParams& parm) const
	{
		float bottomHeight = smin * parm.cellHeight + parm.bottom;
		float topHeight = smax * parm.cellHeight + parm.bottom;
		return std::make_tuple(bottomHeight, topHeight);
	}
	// 修改完毕
	glm::vec3 CylinderSegmentMgr::GetWorldPosFromCellId3D(const CellPos3D& Id, const glm::vec3& Center, const HeightParams& Parm, const glm::vec3& vecDir) const
	{
		glm::vec3 glmCenter(Center.x, Center.y, Center.z);
		CellId cId = static_cast<CellId>(Id);
		glm::vec3 cellPos = listData[CellIdGetIndice(cId)].centerPos;
		glm::vec3 dirVec = -glm::normalize(glm::vec3(cellPos.x, 0, cellPos.z));

		float Length = (Parm.cellHeight * (float(Id.y) + 0.5f));
		float Height = Parm.bottom + Length;

		glm::vec3 toFinPosVec = dirVec * Height;
		glm::vec3 finalPos = cellPos + toFinPosVec + glmCenter;
		auto&& m = ToWorldPosMat(vecDir);
		finalPos = glm::vec3(m * glm::vec4(finalPos, 1.0f));

		return glm::vec3(finalPos.x, finalPos.y, finalPos.z);
	}

	float CylinderSegmentMgr::GetCellId3dDistance(const CellPos3D& s1, const CellPos3D& s2, const HeightParams& parm, DistanceFuncEnum f) const
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
			glm::vec3 p1 = GetWorldPosFromCellId3D(s1, {}, parm);
			glm::vec3 p2 = GetWorldPosFromCellId3D(s2, {}, parm);
			dis = glm::distance(p1, p2);
			break;
		}
		case kSpherical:
		{
			float cos = glm::dot(glm::normalize(c1.centerPos), glm::normalize(c2.centerPos));
			dis = radius * acos(cos);
			break;
		}
		case kSphericalManhattan:
		{
			glm::vec3 p1 = c1.centerPos;
			glm::vec3 p2 = c2.centerPos;
			float dis1 = std::abs(p1.y - p2.y);

			float radians3 = std::abs(atan(p1.z / p1.x));
			float radians4 = std::abs(atan(p2.z / p2.x));
			float radiansSum2 = 0;
			if (p1.z * p2.z < 0)
				radiansSum2 = radians3 + radians4;
			else
				radiansSum2 = std::abs(radians3 - radians4);

			if (radiansSum2 > M_PI)
				radiansSum2 = 2 * M_PI - radiansSum2;

			float dis2 = radius * radiansSum2;

			float disHeight = std::abs((parm.bottom + (parm.cellHeight * (s1.y + 0.5f))) - (parm.bottom + (parm.cellHeight * (s2.y + 0.5f))));

			dis = dis1 + dis2 + disHeight;

			break;
		}
		}
		return dis;
	}
	std::vector<int> CylinderSegmentMgr::GetSquareTileIds(const int TileId1, const int TileId2) const
	{
		std::vector<int> result;
		auto&& [longitude1, Horizontal1] = Get2TileIndexFrom1TileIndex(TileId1);
		auto&& [longitude2, Horizontal2] = Get2TileIndexFrom1TileIndex(TileId2);

		int minLatitudeIndex = std::min(longitude1, longitude2);
		int maxLatitudeIndex = std::max(longitude1, longitude2);

		int minHorizontalIndex = std::min(Horizontal1, Horizontal2);
		int maxHorizontalIndex = std::max(Horizontal1, Horizontal2);
		for (int i = minLatitudeIndex; i <= maxLatitudeIndex; i++)
		{
			for (int j = minHorizontalIndex; j < maxHorizontalIndex; j++)
			{
				result.emplace_back(this->tiles[i][j].tileIndex);
			}
		}
		return result;
	}
	std::vector<int> CylinderSegmentMgr::GetTriangleTileIds(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& center,
		const glm::vec3& vecDir) const
	{
		return std::vector<int>();
	}
	int CylinderSegmentMgr::GetSpanListIndexFromWorldPos(float x, float y, float z) const
	{
		auto&& [longitudeIndex, patchIndex] = Get2TileIndexFromWorldPos(x, y, z);
		const Tile& tile = tiles[longitudeIndex][patchIndex];
		glm::vec3 axis_y = glm::normalize(glm::cross(tile.axis_u, tile.axis_v));
		glm::mat4 matrix = glm::mat4(glm::vec4(tile.axis_u, 0), glm::vec4(axis_y, 0), glm::vec4(tile.axis_v, 0), glm::vec4(0, 0, 0, 1));
		matrix = glm::transpose(matrix);

		glm::vec3 worldPos(x, y, z);

		glm::vec3 SpanListPos = matrix * glm::vec4(worldPos, 1.0f);

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
	int CylinderSegmentMgr::OffsetGetEdgeSpanListNeighborIndex(const Cell& sl, EdgeNeighborDirect e)
	{
		glm::vec3 Center = sl.centerPos;
		glm::mat4 rotate(1.0f);
		if (e == EdgeNeighborDirect::kNegX)
		{
			float HorizontalRadiansStride = 360.0f / float(tiles[0].size() * tileSize) / 180 * float(M_PI);
			rotate = glm::rotate(rotate, HorizontalRadiansStride, glm::vec3(0, 1, 0));
			glm::vec3 pos = rotate * glm::vec4(Center, 1.0f);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}

		if (e == EdgeNeighborDirect::kX)
		{
			float HorizontalRadiansStride = 360.0f / float(tiles[0].size() * tileSize) / 180 * float(M_PI);
			rotate = glm::rotate(rotate, HorizontalRadiansStride, glm::vec3(0, -1, 0));
			glm::vec3 pos = rotate * glm::vec4(Center, 1.0f);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}

		if (e == EdgeNeighborDirect::kNegZ) {
			glm::vec3 pos = Center + glm::vec3(0, -cellSize, 0);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}

		if (e == EdgeNeighborDirect::kZ) {
			glm::vec3 pos = Center + glm::vec3(0, +cellSize, 0);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}

		return 0;
	}

	//修改完毕
	std::tuple<int, int> CylinderSegmentMgr::Get2TileIndexFromWorldPos(float x, float y, float z) const
	{
		float bottom_y = -height / 2.0f;
		int longitude = (y - bottom_y) / (tileSize * cellSize);
		longitude = std::clamp(longitude, 0, int(tiles.size() - 1));

		glm::vec3 PositionVector = { x, y, z };
		int patchIndex = 0;
		float dot = glm::dot(glm::normalize(glm::vec3(PositionVector.x, 0, PositionVector.z)), glm::vec3(1, 0, 0));
		float radians = (float)(PositionVector.z >= 0 ? acos(dot) : 2 * M_PI - acos(dot));
		float HorizontalStride = 2.0f * float(M_PI) / float(tiles[longitude].size());
		patchIndex = radians / HorizontalStride;
		patchIndex = std::clamp(patchIndex, 0, int(tiles[longitude].size() - 1));
		return std::make_tuple(longitude, patchIndex);
	}

	//修改完毕
	glm::mat4 CylinderSegmentMgr::ToWorldPosMat(const glm::vec3& vecDir) const
	{
		if (glm::length(vecDir) < 0.1)
			return glm::mat4(1.0f);
		glm::vec3 dir(vecDir.x, vecDir.y, vecDir.z);
		glm::vec3 axis_z = glm::normalize(glm::cross(glm::vec3(0, 1, 0), dir));
		glm::vec3 axis_x = glm::normalize(glm::cross(dir, axis_z));
		glm::vec3 axis_y = glm::normalize(glm::cross(axis_z, axis_x));
		glm::mat4 mat = glm::mat4(glm::vec4(axis_x, 0.0f), glm::vec4(axis_y, 0), glm::vec4(axis_z, 0), glm::vec4(0, 0, 1, 0));
		return mat;
	}
	std::tuple<glm::vec3, glm::vec3> CylinderSegmentMgr::GetTileWorldAABB(const Tile& t, float top, const glm::vec3& vecDir) const
	{
		const glm::mat4& m = this->ToWorldPosMat(vecDir);
		glm::vec3 center = m * glm::vec4(t.centerPos, 1.0f);
		glm::vec3 axis_u = m * glm::vec4(t.axis_u, 0.0f);
		glm::vec3 axis_v = m * glm::vec4(t.axis_v, 0.0f);

		glm::vec3 v000 = center + (axis_u * cellSize * float(tileSize) / 2.0f) + (axis_v * cellSize * float(tileSize) / 2.0f);
		glm::vec3 v001 = center + (axis_u * cellSize * float(tileSize) / 2.0f) - (axis_v * cellSize * float(tileSize) / 2.0f);
		glm::vec3 v010 = center - (axis_u * cellSize * float(tileSize) / 2.0f) + (axis_v * cellSize * float(tileSize) / 2.0f);
		glm::vec3 v011 = center - (axis_u * cellSize * float(tileSize) / 2.0f) - (axis_v * cellSize * float(tileSize) / 2.0f);

		glm::vec3 axis_y = glm::normalize(glm::cross(axis_u, axis_v));
		glm::vec3 v100 = v000 + (axis_y * top);
		glm::vec3 v101 = v001 + (axis_y * top);
		glm::vec3 v110 = v010 + (axis_y * top);
		glm::vec3 v111 = v011 + (axis_y * top);

		glm::vec3 minPoint = glm::vec3(std::min({ v000.x, v001.x, v010.x, v011.x, v100.x, v101.x, v110.x, v111.x }), std::min({ v000.y, v001.y, v010.y, v011.y, v100.y, v101.y, v110.y, v111.y }),
			std::min({ v000.z, v001.z, v010.z, v011.z, v100.z, v101.z, v110.z, v111.z }));

		glm::vec3 maxPoint = glm::vec3(std::max({ v000.x, v001.x, v010.x, v011.x, v100.x, v101.x, v110.x, v111.x }), std::max({ v000.y, v001.y, v010.y, v011.y, v100.y, v101.y, v110.y, v111.y }),
			std::max({ v000.z, v001.z, v010.z, v011.z, v100.z, v101.z, v110.z, v111.z }));

		return std::make_tuple(minPoint, maxPoint);
	}
}  // namespace segment_mgr
