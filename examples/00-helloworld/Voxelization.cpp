#include"Voxelization.h"
#include "SphereSegmentation.h"
#include "PlaneSegmentation.h"
#include "CylinderSegmentation.h"
#include <array>

#define MaxDepth 20000

namespace voxelFuncs
{
	
	std::vector<std::shared_ptr<wayNode>> findWays(const Span& sp1, const Span& sp2, SegmentMgr& sphere)
	{
		std::shared_ptr<wayNode> start = std::make_shared<wayNode>(0, 0, 0);
		start->sp = &sp1;

		std::shared_ptr<wayNode> end = std::make_shared<wayNode>(0, 0, 0);
		end->sp = &sp2;

		int SearchCount = 0;
		std::vector<std::shared_ptr<wayNode>> open_list;
		std::vector<std::shared_ptr<wayNode>> closed_list;

		open_list.emplace_back(start);
		int count = 0;
		while (!open_list.empty()&&count<10000)
		{
			count++;
			std::shared_ptr<wayNode> current_node = open_list[0];
			int current_index = 0;

			for (int i = 0; i < open_list.size(); ++i)
			{
				if (open_list[i]->f < current_node->f)
				{
					current_node = open_list[i];
					current_index = i;
				}
			}
			closed_list.push_back(current_node);
			std::swap(open_list[current_index], open_list[open_list.size() - 1]);
			open_list.pop_back();
			SearchCount++;
			if (SearchCount > MaxDepth)
			{
				break;
			}

			if (*current_node->sp == *end->sp)
			{
				std::vector<std::shared_ptr<wayNode>> path;
				std::shared_ptr<wayNode> current = current_node;
				while (current != nullptr)
				{
					path.push_back(current);
					current = current->parnet.lock();
				}
				std::reverse(path.begin(), path.end());
				return path;
			}

			std::vector<std::shared_ptr<wayNode>> neighbors;
			auto cellList = sphere.listData;
			auto&& instance = SpanData::getInstance();
			for (int i = 0; i < cellList[current_node->sp->ListIndex].neighborsIndex.size(); i++)
			{
				const auto List = cellList[current_node->sp->ListIndex].neighborsIndex;
				const SpanList& neighborsList = instance.Data[List[i]];
				for (int j = 0; j < neighborsList.Spans.size(); j++)
				{
					auto&& searchSpan = neighborsList.Spans[j];
					if (std::abs(searchSpan.top - current_node->sp->top) > 2* sphere.GetCellSize())
					{
						continue;
					}
					std::shared_ptr<wayNode> newNode = std::make_shared<wayNode>(0, 0, 0);
					newNode->sp = &searchSpan;
					neighbors.emplace_back(newNode);
				}
			}



			for (std::shared_ptr<wayNode> n : neighbors)
			{
				bool inClosedList = false;
				for (int i = 0; i < closed_list.size(); i++)
				{
					if (*closed_list[i]->sp == *n->sp)
					{
						inClosedList = true;
					}
				}
				if (inClosedList)
					continue;

				n->g = current_node->g + sphere.GetCellSize();
				n->h = getSpanDistance(*n->sp, *end->sp, sphere);
				//n->g = current_node->g + getSpantDistanceManhattan(*n->sp, *current_node->sp, sphere);
				//n->h = getSpantDistanceManhattan(*n->sp, *end->sp, sphere);
				n->f = n->g + n->h;
				n->parnet = current_node;

				open_list.push_back(n);
			}
		}

		return std::vector<std::shared_ptr<wayNode>>();

	}

	Span getRandomSpan(const SegmentMgr& Sphere)
	{
		int beginIndex = SpanData::getInstance().Data[0].TileIndex;
		int endIndex = SpanData::getInstance().Data[SpanData::getInstance().Data.size()-1].TileIndex;
		int range = endIndex - beginIndex + 1;
		int RandomListIndex;
		int RandomSpanIndex;
		while (true)
		{
			RandomListIndex = rand() % range + beginIndex;
			
			if (SpanData::getInstance().Data[RandomListIndex].Spans.size() > 0)
			{
				RandomSpanIndex = rand() % (SpanData::getInstance().Data[RandomListIndex].Spans.size());
				break;
			}
		}
		return  SpanData::getInstance().Data[RandomListIndex].Spans[RandomSpanIndex];
	}
	


	void ReCastSphereVoxelization(const std::unique_ptr<SceneMgr>& dataPtr, const std::shared_ptr<segment_mgr::SegmentMgr> Sphere, int TileSize, float cellStride, float cellHeight, float minHeight, float maxHeight)
	{
		auto&& instance = SpanData::getInstance();
		auto&& Data = instance.Data;
		Data.reserve(Sphere->GetTotalTileNums() * TileSize * TileSize);
		for (int i = 0; i < Sphere->GetTotalTileNums(); i++)
		{
			ReCastSingleTileReCast(Sphere->GetTileByIndex(i), dataPtr, Sphere, TileSize, cellStride, cellHeight, minHeight, maxHeight);
		}
		for (int i = 0; i < Sphere->listData.size(); i++)
		{
			Data[i].CenteralWorldPos = Sphere->listData[i].centerPos;
		}
	}
	
	void ReCastSingleTileReCast(const Tile& tile, const std::unique_ptr<SceneMgr>& dataPtr, const std::shared_ptr<segment_mgr::SegmentMgr> Sphere, int TileSize, float cellStride, float cellHeight, float minHeight, float maxHeight)
	{
		glm::vec3 axis_y;
		glm::vec3 MinPoint, MaxPoint;
		std::shared_ptr<SphereSegmentMgr> ssm = std::dynamic_pointer_cast<SphereSegmentMgr>(Sphere);
		std::shared_ptr<PlaneSegmentMgr> psm = std::dynamic_pointer_cast<PlaneSegmentMgr>(Sphere);
		std::shared_ptr<CylinderSegmentMgr> csm = std::dynamic_pointer_cast<CylinderSegmentMgr>(Sphere);
		if (ssm != nullptr)
		{
			axis_y = glm::normalize(tile.centerPos);
			auto&& tup = ssm->GetTileSectorWorldAABB(tile,ssm->GetRadius(),ssm->GetRadius()+maxHeight);
			MinPoint = std::get<0>(tup);
			MaxPoint = std::get<1>(tup);
			float near = 0.1;
			float far  =  maxHeight;
		}
		else if (psm != nullptr)
		{
			axis_y = glm::vec3(0, 1, 0);
			auto&& tup = psm->GetTileWorldAABB(tile, maxHeight);
			MinPoint = std::get<0>(tup);
			MaxPoint = std::get<1>(tup);
		}
		else if (csm != nullptr)
		{
			axis_y = -glm::normalize(glm::vec3(tile.centerPos.x,0,tile.centerPos.z));
			auto&& tup = csm->GetTileWorldAABB(tile, csm->GetRadius(),{});
			MinPoint = std::get<0>(tup);
			MaxPoint = std::get<1>(tup);
		}
		
		std::vector<MeshObject> GeosInthisTile;
		rcHeightfield* HeightField = nullptr;
		HeightField = rcAllocHeightfield();
		
		float HeightFiledMin[3] = { -TileSize * cellStride / 2.0f, minHeight,-TileSize * cellStride / 2.0f };
		float HeightFiledMax[3] = { TileSize * cellStride / 2.0f, maxHeight ,TileSize * cellStride / 2.0f };
		rcCreateHeightfield(nullptr, *HeightField, TileSize, TileSize, HeightFiledMin, HeightFiledMax, cellStride, cellHeight);

		for (int i = 0; i < dataPtr->MeshObjects.size(); i++)
		{
			const MeshObject& GD = dataPtr->MeshObjects[i];
			if (wetherAABBIntersect(GD.WorldMin, GD.WorldMax, MinPoint, MaxPoint))
			{
				GeosInthisTile.emplace_back(GD);
			}
		}

		for (int i = 0; i < GeosInthisTile.size(); i++)
		{
			glm::mat4 toTilematrix = glm::mat4(glm::vec4(tile.axis_u, 0), glm::vec4(axis_y, 0), glm::vec4(tile.axis_v, 0), glm::vec4(0, 0, 0, 1));
			toTilematrix = glm::transpose(toTilematrix);
			glm::mat4 toTilematrixRevers = glm::transpose(toTilematrix);

			glm::mat4 Worldm;
			Worldm = glm::translate(glm::mat4(1.0f), GeosInthisTile[i].WorldPos) * glm::toMat4(GeosInthisTile[i].rotation) * glm::scale(glm::mat4(1.0f), GeosInthisTile[i].scale);

			std::shared_ptr<MeshData> MeshD = dataPtr->MeshMap[GeosInthisTile[i].MeshPathName];
			std::vector<float> vecs;
			vecs.reserve(MeshD->worldVertices.size() * 3);
			for (int index = 0; index < MeshD->worldVertices.size(); index++)
			{
				glm::vec3 worldPos = Worldm * glm::vec4(MeshD->worldVertices[index], 1.0f);
				glm::vec4 PosInTilexyz= toTilematrix * glm::vec4(worldPos - tile.centerPos, 1.0f);
				float length = -PosInTilexyz.y + 1500.0f;
				vecs.emplace_back(PosInTilexyz.x / length * 1500.0f);
				vecs.emplace_back(PosInTilexyz.y);
				vecs.emplace_back(PosInTilexyz.z / length * 1500.0f);
			}
			static thread_local rcContext ctx;
			auto triareas = std::make_unique<unsigned char[]>(MeshD->indices.size() / 3);
			memset(triareas.get(), RC_WALKABLE_AREA, MeshD->indices.size() / 3 * sizeof(unsigned char));
			rcRasterizeTriangles (&ctx, vecs.data(), MeshD->worldVertices.size(), MeshD->indices.data(), triareas.get(), MeshD->indices.size() / 3, *HeightField, 10000);
			vecs.swap(std::vector<float>());
		}

		ReCastHeightFieldToSpanData(tile, *HeightField, Sphere, cellHeight, minHeight);
		rcFreeHeightField(HeightField);
		HeightField = nullptr;
	}
	
	void ReCastHeightFieldToSpanData(const Tile& t, rcHeightfield& hf, const std::shared_ptr<segment_mgr::SegmentMgr> Sphere,float cellHeight,float minHeight)
	{
		auto&& instance = SpanData::getInstance();
		for (int x = 0; x < Sphere->GetTileSize(); x++)
		{
			for (int z = 0; z < Sphere->GetTileSize(); z++)
			{
				SpanList List({ 0,0,0 }, t.tileIndex, 0);
				int Index = x + (uint64_t)z * hf.width;
				rcSpan* s = hf.spans[Index];
				if (s)
				{
					while (s)
					{
						List.Spans.emplace_back(s->smin * cellHeight + minHeight, s->smax * cellHeight + minHeight, instance.Data.size()+1);
						s = s->next;
					}
				}
				instance.Data.emplace_back(std::move(List));
			}
		}
	}

	
	float getSpanDistance(const Span& s1, const Span& s2, SegmentMgr& sphere)
	{
		auto&& instance = SpanData::getInstance();
		
		
		auto&& Tile1 = sphere.GetTileByIndex(instance.Data[s1.ListIndex].TileIndex);
		auto&& Tile2 = sphere.GetTileByIndex(instance.Data[s2.ListIndex].TileIndex);

		glm::vec3 axis_y1 = glm::normalize(glm::cross(Tile1.axis_u, Tile1.axis_v));
		glm::vec3 axis_y2 = glm::normalize(glm::cross(Tile2.axis_u, Tile2.axis_v));

		glm::vec3 p1 = instance.Data[s1.ListIndex].CenteralWorldPos + (axis_y1 * s1.top);
		glm::vec3 p2 = instance.Data[s2.ListIndex].CenteralWorldPos + (axis_y2 * s2.top);
		

		p1 = instance.Data[s1.ListIndex].CenteralWorldPos;
		p2 = instance.Data[s2.ListIndex].CenteralWorldPos;
		float radians1 = asin(p1.y / 1500.0f);
		float radians2 = asin(p2.y / 1500.0f);

		float radiansSum1 = 0;
		if (p1.y * p2.y < 0)
			radiansSum1 = radians1 + radians2;
		else
			radiansSum1 = std::abs(radians1 - radians2);

		if (radiansSum1 > M_PI)
			radiansSum1 = 2 * M_PI - radiansSum1;

		float dis1 = 1500.0f * radiansSum1;

		float radius2 = std::sqrt((1500.0f * 1500.0f) - (p1.y * p1.y));

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

		float dis = dis1 + dis2;

		return dis;
		//return glm::distance(p1, p2);
	}

	/*
	float getSpantDistanceManhattan(const Span& s1, const Span& s2, SegmentMgr& sphere)
	{
		auto&& instance = SpanData::getInstance();
		auto&& SpanList1 = instance.Data[s1.ListIndex];
		auto&& SpanList2 = instance.Data[s2.ListIndex];

		glm::vec3 p1 = SpanList1.CenteralWorldPos - sphere.CenterPos;
		glm::vec3 p2 = SpanList2.CenteralWorldPos - sphere.CenterPos;
		glm::vec3 temp(p2.x, p1.y, p2.z);
		float cos1 = glm::dot(glm::normalize(p1), glm::normalize(temp));
		float dis1 = sphere.Radius * acos(cos1);
		float cos2 = glm::dot(glm::normalize(temp), glm::normalize(p2));
		float dis2 = sphere.Radius * acos(cos2);
		return dis1 + dis2;

	}
	*/
}
