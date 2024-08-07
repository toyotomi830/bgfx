#include "PlaneSegmentation.h"

namespace segment_mgr
{
	PlaneSegmentMgr::PlaneSegmentMgr(float _stride, int _tileSize, float _cellSize):stride(_stride)
	{
		tileSize = _tileSize;
		cellSize = _cellSize;
		if ((int)stride % (int)(tileSize * cellSize) != 0)
		{
			stride = float((int)(stride / (tileSize * cellSize)) + 1)* (tileSize * cellSize);
		}
		int SIZE = (int)(stride / (tileSize * cellSize));
		tiles.resize(SIZE);
		glm::vec3 start(-stride / 2.0f, 0, -stride / 2.0f);
		for (int i = 0; i < SIZE; i++)
		{
			for (int j = 0; j < SIZE; j++)
			{
				
				glm::vec3 tileCenteral = start + glm::vec3(((float)i + 0.5f) * tileSize * cellSize,0,0) + glm::vec3(0,0,((float)j + 0.5f) * tileSize * cellSize);
				Tile t;
				t.axis_u = glm::vec3(1, 0, 0);
				t.axis_v = glm::vec3(0, 0, -1);
				t.tileIndex = totalTilesNum;
				t.centerPos = tileCenteral;
				tiles[i].emplace_back(std::move(t));
				totalTilesNum++;
			}
		}

		listData.reserve(totalTilesNum * tileSize * tileSize);
		handleData.reserve(totalTilesNum * tileSize * tileSize);

		for (auto&& i : tiles)
		{
			for (auto&& j : i)
			{
				glm::vec3 centerPoint = j.centerPos;
				glm::vec3 MinP = centerPoint + glm::vec3(-(float)tileSize * cellSize/2.0f,0, -(float)tileSize * cellSize / 2.0f);
				for (int x = 0; x < tileSize; x++)
				{
					for (int z = 0; z < tileSize; z++)
					{
						glm::vec3 ListMinPoint = MinP + glm::vec3(float(x) * cellSize, 0, 0) + glm::vec3(0, 0, float(z) * cellSize);
						glm::vec3 ListCenterPoint = ListMinPoint + glm::vec3(cellSize / 2.0f,0,0) + glm::vec3(0, 0,cellSize / 2.0f);
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
	int PlaneSegmentMgr::GetTileIndexFromWorldPos(const glm::vec3& xyz, const glm::vec3& center, const glm::vec3& vecDir) const
	{
		auto&& [x,z] = Get2TileIndexFromWorldPos(xyz.x - center.x, xyz.y - center.y, xyz.z - center.z);
		return tiles[x][z].tileIndex;
	}
	const CellPos3D PlaneSegmentMgr::GetCellId3DFromWorldPos(const glm::vec3& xyz, const glm::vec3& center, const HeightParams& parm, const glm::vec3& vecDir) const
	{
		auto&& temp = xyz - center;
		auto id = handleData[GetSpanListIndexFromWorldPos(temp.x, temp.y, temp.z)];
		float length = std::abs(temp.y);

		float div = length - parm.bottom;
		uint16_t y = (uint16_t)(div / parm.cellHeight);

		auto&& pos = listData[CellIdGetIndice(id)].centerPos;
		return CellPos3D(id.tileId, id.x, y, id.z);
	}
	std::tuple<float, float> PlaneSegmentMgr::GetSpanHeight(unsigned int smin, unsigned smax, const HeightParams& parm) const
	{
		float bottomHeight = smin * parm.cellHeight + parm.bottom ;
		float topHeight = smax * parm.cellHeight + parm.bottom;
		return std::make_tuple(bottomHeight, topHeight);
	}

	glm::vec3 PlaneSegmentMgr::GetWorldPosFromCellId3D(const CellPos3D& Id, const glm::vec3& Center, const HeightParams& Parm,const glm::vec3& vecDir) const
	{
		glm::vec3 glmCenter(Center.x, Center.y, Center.z);
		CellId cId = static_cast<CellId>(Id);
		glm::vec3 cellPos = listData[CellIdGetIndice(cId)].centerPos;
		cellPos = cellPos + glmCenter;

		float Length = (Parm.cellHeight * (float(Id.y) + 0.5f));
		float Height = (Parm.bottom + Length);

		glm::vec3 finalPos = cellPos + glm::vec3(0, Height, 0);
		return glm::vec3(finalPos.x, finalPos.y, finalPos.z);
	}

	float PlaneSegmentMgr::GetCellId3dDistance(const CellPos3D& s1, const CellPos3D& s2, const HeightParams& parm, DistanceFuncEnum f) const
	{
		return 0.0f;
	}

	std::vector<int> PlaneSegmentMgr::GetSquareTileIds(const int TileId1, const int TileId2) const
	{
		return std::vector<int>();
	}

	std::vector<int> PlaneSegmentMgr::GetTriangleTileIds(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& center, const glm::vec3& vecDir) const
	{
		return std::vector<int>();
	}

	int PlaneSegmentMgr::GetSpanListIndexFromWorldPos(float x, float y, float z) const
	{
		auto&& [i, j] = Get2TileIndexFromWorldPos(x, y, z);
		const Tile& tile = tiles[i][j];
		glm::vec3 centerPos = tile.centerPos;
		glm::vec3 Minp = tile.centerPos + glm::vec3(-(float)tileSize * cellSize / 2.0f, 0, -(float)tileSize * cellSize / 2.0f);

		int dividex = (x - Minp.x) / (cellSize);
		dividex = std::clamp(dividex, 0, tileSize - 1);

		int dividez = (z - Minp.z) / (cellSize);
		dividez = std::clamp(dividez, 0, tileSize - 1);

		return tile.tileIndex * tileSize * tileSize + dividex * tileSize + dividez;
	}

	int PlaneSegmentMgr::OffsetGetEdgeSpanListNeighborIndex(const Cell& sl, EdgeNeighborDirect e)
	{
		glm::vec3 Center = sl.centerPos;
		if (e == EdgeNeighborDirect::kNegX)
		{
			glm::vec3 pos = Center + glm::vec3(-cellSize, 0, 0);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}

		if (e == EdgeNeighborDirect::kX)
		{
			glm::vec3 pos = Center + glm::vec3(+cellSize, 0, 0);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}

		if (e == EdgeNeighborDirect::kNegZ)
		{
			glm::vec3 pos = Center + glm::vec3(0, 0, -cellSize);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}

		if (e == EdgeNeighborDirect::kZ)
		{
			glm::vec3 pos = Center + glm::vec3(0, 0, +cellSize);
			return GetSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
		}

		return 0;
	}

	std::tuple<int, int> PlaneSegmentMgr::Get2TileIndexFromWorldPos(float x, float y, float z) const
	{
		glm::vec3 Minp(-stride / 2.0f, 0, -stride / 2.0f);
		int SIZE = stride / (tileSize * cellSize);
		int dividex = (x - Minp.x) / (cellSize * (float)tileSize);
		dividex = std::clamp(dividex, 0, SIZE - 1);
		int dividez = (z - Minp.z) / (cellSize * (float)tileSize);
		dividez = std::clamp(dividez, 0, SIZE - 1);
		return std::make_tuple(dividex, dividez);
	}
}

