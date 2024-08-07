#include "SegmentationBase.h"

namespace segment_mgr
{
	const Tile& SegmentMgr::GetTileByIndex(int index) const
	{
		for (int longitudeIndex = (int)tiles.size() - 1; longitudeIndex >= 0; longitudeIndex -= 1)
		{
			if (tiles[longitudeIndex][0].tileIndex <= index)
			{
				for (std::size_t i = 0; i < tiles[longitudeIndex].size(); i++)
				{
					if (tiles[longitudeIndex][i].tileIndex == index) return tiles[longitudeIndex][i];
				}
			}
		}
		return tiles[0][0];
	}

	std::vector<CellId> SegmentMgr::GetCellIdNeighbors(CellId& sph)const
	{
		std::vector<CellId> result;
		auto&& List = listData[CellIdGetIndice(sph)];
		for (std::size_t i = 0; i < List.neighborsIndex.size(); i++) {
			result.emplace_back(handleData[List.neighborsIndex[i]]);
		}
		return result;
	}

	std::tuple<glm::vec3, glm::vec3> SegmentMgr::GetTileWorldAABB(const Tile& t, float top) const
	{
		glm::vec3 center = t.centerPos;
		glm::vec3 v000 = t.centerPos + (t.axis_u * cellSize * float(tileSize) / 2.0f) + (t.axis_v * cellSize * float(tileSize) / 2.0f);
		glm::vec3 v001 = t.centerPos + (t.axis_u * cellSize * float(tileSize) / 2.0f) - (t.axis_v * cellSize * float(tileSize) / 2.0f);
		glm::vec3 v010 = t.centerPos - (t.axis_u * cellSize * float(tileSize) / 2.0f) + (t.axis_v * cellSize * float(tileSize) / 2.0f);
		glm::vec3 v011 = t.centerPos - (t.axis_u * cellSize * float(tileSize) / 2.0f) - (t.axis_v * cellSize * float(tileSize) / 2.0f);

		glm::vec3 axis_y = glm::normalize(glm::cross(t.axis_u, t.axis_v));
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

	std::tuple<glm::vec3, glm::vec3> SegmentMgr::GetTileSectorWorldAABB(const Tile& t, float near,float far) const
	{
		float stride = (cellSize * tileSize / 2.0f) / near * far;

		glm::vec3 v000 = t.centerPos + (t.axis_u * stride) + (t.axis_v * stride);
		glm::vec3 v001 = t.centerPos + (t.axis_u * stride) - (t.axis_v * stride);
		glm::vec3 v010 = t.centerPos - (t.axis_u * stride) + (t.axis_v * stride);
		glm::vec3 v011 = t.centerPos - (t.axis_u * stride) - (t.axis_v * stride);

		glm::vec3 axis_y = glm::normalize(glm::cross(t.axis_u, t.axis_v));
		glm::vec3 v100 = v000 + (axis_y * (far - near));
		glm::vec3 v101 = v001 + (axis_y * (far - near));
		glm::vec3 v110 = v010 + (axis_y * (far - near));
		glm::vec3 v111 = v011 + (axis_y * (far - near));

		glm::vec3 minPoint = glm::vec3(std::min({ v000.x, v001.x, v010.x, v011.x, v100.x, v101.x, v110.x, v111.x }), std::min({ v000.y, v001.y, v010.y, v011.y, v100.y, v101.y, v110.y, v111.y }),
			std::min({ v000.z, v001.z, v010.z, v011.z, v100.z, v101.z, v110.z, v111.z }));

		glm::vec3 maxPoint = glm::vec3(std::max({ v000.x, v001.x, v010.x, v011.x, v100.x, v101.x, v110.x, v111.x }), std::max({ v000.y, v001.y, v010.y, v011.y, v100.y, v101.y, v110.y, v111.y }),
			std::max({ v000.z, v001.z, v010.z, v011.z, v100.z, v101.z, v110.z, v111.z }));
		return std::make_tuple(minPoint, maxPoint);
	}

	std::tuple<glm::vec3, glm::vec3> SegmentMgr::GetCell3DAabb(const CellPos3D& c3d, const HeightParams& parm) const
	{
		const Cell& c = listData[CellIdGetIndice(c3d)];
		const Tile& t = GetTileByIndex(c3d.tileId);
		glm::vec3 axis_y = glm::normalize(glm::cross(t.axis_u, t.axis_v));

		glm::vec3 v000 = c.centerPos + (t.axis_u * cellSize / 2.0f) + (t.axis_v * cellSize / 2.0f) + (axis_y * parm.cellHeight * (float)c3d.y);
		glm::vec3 v001 = c.centerPos + (t.axis_u * cellSize / 2.0f) - (t.axis_v * cellSize / 2.0f) + (axis_y * parm.cellHeight * (float)c3d.y);
		glm::vec3 v010 = c.centerPos - (t.axis_u * cellSize / 2.0f) + (t.axis_v * cellSize / 2.0f) + (axis_y * parm.cellHeight * (float)c3d.y);
		glm::vec3 v011 = c.centerPos - (t.axis_u * cellSize / 2.0f) - (t.axis_v * cellSize / 2.0f) + (axis_y * parm.cellHeight * (float)c3d.y);


		glm::vec3 v100 = v000 + (axis_y * parm.cellHeight);
		glm::vec3 v101 = v001 + (axis_y * parm.cellHeight);
		glm::vec3 v110 = v010 + (axis_y * parm.cellHeight);
		glm::vec3 v111 = v011 + (axis_y * parm.cellHeight);


		glm::vec3 minPoint = glm::vec3(std::min({ v000.x, v001.x, v010.x, v011.x, v100.x, v101.x, v110.x, v111.x }), std::min({ v000.y, v001.y, v010.y, v011.y, v100.y, v101.y, v110.y, v111.y }),
			std::min({ v000.z, v001.z, v010.z, v011.z, v100.z, v101.z, v110.z, v111.z }));

		glm::vec3 maxPoint = glm::vec3(std::max({ v000.x, v001.x, v010.x, v011.x, v100.x, v101.x, v110.x, v111.x }), std::max({ v000.y, v001.y, v010.y, v011.y, v100.y, v101.y, v110.y, v111.y }),
			std::max({ v000.z, v001.z, v010.z, v011.z, v100.z, v101.z, v110.z, v111.z }));

		return std::make_tuple(minPoint, maxPoint);
	}

	std::tuple<int, int> SegmentMgr::Get2TileIndexFrom1TileIndex(int Index) const
	{
		for (int longitudeIndex = (int)tiles.size() - 1; longitudeIndex >= 0; longitudeIndex -= 1)
		{
			if (tiles[longitudeIndex][0].tileIndex <= Index)
			{
				for (std::size_t i = 0; i < tiles[longitudeIndex].size(); i++)
				{
					if (tiles[longitudeIndex][i].tileIndex == Index)
						return std::make_tuple((int)longitudeIndex, (int)i);
				}
			}
		}
		return std::tuple<int, int>(0, 0);
	}
}
