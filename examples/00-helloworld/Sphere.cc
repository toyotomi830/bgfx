#include "Sphere.hh"

namespace segment
{
    SphereSegmentation::SphereSegmentation(float _radius, int _tileSize, float _cellSize)
    {
		radius = _radius;
		tileSize = _tileSize;
		cellSize = _cellSize;
		unitRadianSize_ = float(M_PI) / (float(M_PI) * radius / (cellSize * tileSize)); //rad for each cell;
		glm::mat4 m(1.0f);
		m = glm::scale(glm::mat4(1.0f), glm::vec3(radius, radius, radius));
		int N = int(float(M_PI) / unitRadianSize_ + 2.0f);
		tileList.resize(N + 1);
		float dl = 180.0f / N;
		for (int i = 0; i <= N; i++)
		{
			float degree = -90.0f + i * dl;
			float rad = degree / 360.0f * float(M_PI) * 2.0f;
			float y = sinf(rad);
			float r = cosf(rad);
			float perimeter = r * float(M_PI) * 2;
			if (perimeter <= unitRadianSize_) {//polar point
				tileList[i].resize(1);
				Tile& t = tileList[i][0];
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
				t.tileIndex = tileNums;
				tileNums++;
			}
			else {
				int NUM = int(perimeter / unitRadianSize_ + 4.0f);
				tileList[i].resize(NUM);
				float da = float(M_PI) * 2.0f / NUM;
				for (int j = 0; j < NUM; ++j) {//put tile on each latitude
					float lat = da * j;
					float x = r * cosf(lat);
					float z = r * sinf(lat);
					Tile& t = tileList[i][j];
					t.centerPos = { x, y, z };
					t.axis_u = glm::normalize(glm::cross(t.centerPos, glm::vec3(0, 1, 0)));
					t.axis_v = glm::normalize(glm::cross(t.centerPos, t.axis_u));
					t.centerPos = m * glm::vec4(t.centerPos, 1.0f);
					t.tileIndex = tileNums;
					tileNums++;
				}
			}
		}
		cellList.reserve(tileNums * tileSize * tileSize);
		for each (auto&& var in tileList)
		{
			for each (Tile tile in var)
			{
				glm::vec3 centerPoint = tile.centerPos;
				glm::vec3 u = tile.axis_u;
				glm::vec3 v = tile.axis_v;
				glm::vec3 negu = -u;
				glm::vec3 negv = -v;
				glm::vec3 toMin = (negu * cellSize * float(tileSize) / 2.0f) + (negv * cellSize * float(tileSize) / 2.0f);
				glm::vec3 MinPos = centerPoint + toMin;
				for (int x = 0; x < tileSize; x++)
				{
					for (int z = 0; z < tileSize; z++)
					{
						glm::vec3 ListMinPoint = MinPos + (u * float(x) * cellSize) + (v * float(z) * cellSize);
						glm::vec3 ListCenterPoint = ListMinPoint + (u * cellSize / 2.0f) + v * float(cellSize / 2.0f);
						auto overlayTiles = getOverLayTile(ListCenterPoint);//get every tile match with this position;
						CellId newId(tile.tileIndex, x, z);
						if (overlayTiles.size() == 1) {// this is the only tile, create cell;
							Cell newCell;
							newCell.CenteralWorldPos = ListMinPoint;
							newCell.id = newId;
							cellData.push_back(newCell);
							cellList[CellIdGetIndice(newId)] = &cellData[cellData.size() - 1];
						}
						else
						{
							bool firstTile = true;
							for each (auto&& indexSet in overlayTiles)
							{
								int latitude = std::get<0>(indexSet);
								int longitude = std::get<1>(indexSet);
								if (tileList[latitude][longitude].tileIndex <= tile.tileIndex) {//this position appeared in past tile, save pointer to first cell;
									cellList[CellIdGetIndice(newId)] = cellList[getCellIndexFormTile(tileList[latitude][longitude],ListCenterPoint)];
									cellList[CellIdGetIndice(newId)]->id = newId;
									firstTile = false;
									break;
								}
							}
							if (firstTile) {//is the first Tile for this position appear, create new cell
								Cell newCell;
								newCell.CenteralWorldPos = ListMinPoint;
								newCell.id = newId;
								cellData.push_back(newCell);
								cellList[CellIdGetIndice(newId)] = &cellData[cellData.size() - 1];
							}
						}
					}
				}
			}
		}
		findNeighbor();
    }

    int SphereSegmentation::GetLatitudeIndex(float x, float y, float z) const
    {
        glm::vec3 pos = { x, y, z };
        return GetLatitudeIndex(pos);
		
    }

    int SphereSegmentation::GetLatitudeIndex(glm::vec3 pos) const
    {
		float dot = glm::dot(glm::normalize(pos), glm::vec3(0, -1, 0));
		float angle = acos(dot);

		int latitudeIndex = 0;
		int patchIndex = 0;

		if ((M_PI - angle) < atan((GetCellSize() * (float)tileSize / 2.0f) / radius))//check if point at top tile
		{
			latitudeIndex = (int)(tileList.size() - 1);
		}
		else
		{
			int tempIndex = (int)(angle / latitudeStride);

			float right = latitudeStride * float(tempIndex + 1);

			latitudeIndex = (right - angle) < atan((GetCellSize() * (float)tileSize / 2.0f) / radius) ? tempIndex + 1 : tempIndex;//rounding latitude
		}
		return latitudeIndex;
    }

    int SphereSegmentation::OffsetGetEdgeSpanListNeighborIndex(const Cell &sl, NeighborDirection direction)
    {
		glm::vec3 Center = sl.CenteralWorldPos;
		int latitude = GetLatitudeIndex(Center.x, Center.y, Center.z);
		glm::mat4 rotate(1.0f);
		glm::vec3 pos;
		float rotationStride = 2.0f * float(M_PI) / float(tileList[latitude].size());
		switch (direction)
		{
		case NeighborDirection::POSY: {
			rotate = glm::rotate(rotate, rotationStride, glm::vec3(0, 1, 0));
			pos = rotate * glm::vec4(Center, 1.0f);
		} break;
		case NeighborDirection::NEGY: {
			rotate = glm::rotate(rotate, rotationStride, glm::vec3(0, -1, 0));
			pos = rotate * glm::vec4(Center, 1.0f);
		} break;
		case NeighborDirection::POSX:  {
			glm::vec3 rotateAxis = glm::normalize(glm::cross((Center), glm::vec3(0, 1, 0)));
			rotate = glm::rotate(rotate, rotationStride, rotateAxis);
			pos = rotate * glm::vec4(Center, 1.0f);
		} break;
		case NeighborDirection::NEGX:  {
			glm::vec3 rotateAxis = glm::normalize(glm::cross((Center), glm::vec3(0, -1, 0)));
			rotate = glm::rotate(rotate, rotationStride, rotateAxis);
			pos = rotate * glm::vec4(Center, 1.0f);
		} break;
		default:
			break;
		}
		return GetCellIndex(pos);
    }

    void SphereSegmentation::findNeighbor()
    {
        for (auto && i:tileList)
        {
            for (auto && j:i)
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
						int cellIndex = j.tileIndex * tileSize * tileSize + x * tileSize + z;
						int neighbor;
						Cell* cell = cellList[cellIndex];
						if (x != 0)
						{
							cell->neighborIndex[0] = cellIndex - 1;
						}
						else
						{
							cell->neighborIndex[0] = (OffsetGetEdgeSpanListNeighborIndex(*cell, NeighborDirection::POSY));
						}
						if (x != tileSize - 1)
						{
							cell->neighborIndex[2] = (cellIndex + 1);
						}
						else
						{
							cell->neighborIndex[2] = (OffsetGetEdgeSpanListNeighborIndex(*cell, NeighborDirection::NEGY));
						}
						if (z != 0)
						{
							cell->neighborIndex[1] = (cellIndex - tileSize);
						}
						else
						{
							cell->neighborIndex[1] = (OffsetGetEdgeSpanListNeighborIndex(*cell, NeighborDirection::POSX));
						}
						if (z != tileSize - 1)
						{
							cell->neighborIndex[3] = (cellIndex + tileSize);
						}
						else
						{
							cell->neighborIndex[3] = (OffsetGetEdgeSpanListNeighborIndex(*cell, NeighborDirection::NEGX));
						}
					}
				}
			}
        }
    }

	std::vector<std::tuple<int, int>> SphereSegmentation::getOverLayTile(const glm::vec3& xyz)
	{
		return std::vector<std::tuple<int, int>>();
	}

	int SphereSegmentation::getCellIndexFormTile(Tile, glm::vec3)
	{
		return 0;
	}
        
    int SphereSegmentation::GetTileIndex(const glm::vec3 &xyz, const glm::vec3 &vecDir) const
    {
        auto&& [latitude,longitude]=GetTileIndex2(xyz,vecDir);
        return tileList[latitude][longitude].tileIndex;
    }

    std::tuple<int,int> SphereSegmentation::GetTileIndex2(const glm::vec3 &xyz, const glm::vec3 &vecDir) const
    {
        glm::vec3 pos= xyz - center;
        int latitude = GetLatitudeIndex(pos);
        if(latitude == 0 || latitude == int(tileList.size()-1)){
			return std::make_tuple(latitude, 0);
        }
        int longitude = 0;
		float longitudeStride = 2.0f * float(M_PI) / float(tileList[latitude].size());
        float dot1 = glm::dot(glm::normalize(pos), glm::vec3(0, 1, 0));
        float dot2 = glm::dot(glm::vec3(pos.x, 0, pos.z), glm::vec3(1, 0, 0));
        float radians = (float)(pos.z >= 0 ? acos(dot2) : 2 * M_PI - acos(dot2));
        float radius2 = std::abs(radius * sin(acos(dot1)));
        float stride = atan((GetCellSize() * (float)tileSize / 2.0f) / radius2);
        if ((2 * M_PI - radians) < stride / 2.0)
		{
			return std::make_tuple(latitude, 0);
		}
		else
		{
			int tempIndex = (int)(radians / longitudeStride);
			float right = float(tempIndex + 1) * longitudeStride;
			longitude = (right - radians) < stride ? tempIndex + 1 : tempIndex;
		}
		if (longitude == (int)tileList[latitude].size())
		{
			longitude = 0;
		}
		return std::make_tuple(latitude, longitude);
    }

    float SphereSegmentation::GetCellDistance(const CellSpanId start, const CellSpanId end, const HeightParams param)
    {
		const Cell* startCell = cellList[std::get<0>(start)];
		const Cell* endCell = cellList[std::get<0>(end)];
		const Span startSpan = startCell->spans[std::get<1>(start)];
		const Span endSpan = startCell->spans[std::get<1>(end)];

		glm::vec3 startPos = startCell->CenteralWorldPos;
		glm::vec3 endPos = endCell->CenteralWorldPos;

		float radians1 = asin(startPos.y / radius);
		float radians2 = asin(endPos.y / radius);
		float radiansSum1 = 0;

		if (startPos.y * endPos.y < 0)
			radiansSum1 = radians1 + radians2;
		else
			radiansSum1 = std::abs(radians1 - radians2);

		if (radiansSum1 > M_PI)
			radiansSum1 = 2.0f * M_PI - radiansSum1;

		float dis1 = radius * radiansSum1;

		float radius2 = std::sqrt((radius * radius) - (startPos.y * startPos.y));

		float radians3 = atan(startPos.z / startPos.x);
		float radians4 = atan(endPos.z / endPos.x);
		float radiansSum2 = 0;
		if (endPos.z * startPos.z < 0)
			radiansSum2 = radians3 + radians4;
		else
			radiansSum2 = std::abs(radians3 - radians4);

		if (radiansSum2 > M_PI)
			radiansSum2 = 2.0f * M_PI - radiansSum2;

		float dis2 = radius2 * radiansSum2;

		float disHeight = std::abs(startSpan.top - endSpan.top);
        return dis1+dis2+disHeight;
    }

	int SphereSegmentation::GetCellIndex(const glm::vec3 cellPos) const
	{
		auto&& [latitude, longitude] = GetTileIndex2(cellPos);
		const Tile& tile = tileList[latitude][longitude];
		glm::vec3 axis_y = glm::normalize(glm::cross(tile.axis_u, tile.axis_v));
		glm::mat4 matrix = glm::mat4(glm::vec4(tile.axis_u, 0), glm::vec4(axis_y, 0), glm::vec4(tile.axis_v, 0), glm::vec4(0, 0, 0, 1));
		matrix = glm::transpose(matrix);

		glm::vec3 worldPos = cellPos;
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

} // namespace segment
