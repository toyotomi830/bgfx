#pragma once
#include "SegmentationBase.h"

namespace segment_mgr
{
	class SphereSegmentMgr : public SegmentMgr
	{
	public:
		//------------------------------Virtual func
		int GetTileIndexFromWorldPos(const glm::vec3& xyz, const glm::vec3& center = {}, const glm::vec3& vecDir = {}) const final;
		const CellPos3D GetCellId3DFromWorldPos(const glm::vec3& xyz, const glm::vec3& center = {}, const HeightParams& parm = {}, const glm::vec3& vecDir = {}) const final;
		std::tuple<float, float> GetSpanHeight(unsigned int smin, unsigned smax, const HeightParams& parm) const final;
		glm::vec3 GetWorldPosFromCellId3D(const CellPos3D& Id, const glm::vec3& Center, const HeightParams& Parm, const glm::vec3& vecDir = {}) const final;
		float GetCellId3dDistance(const CellPos3D& s1, const CellPos3D& s2, const HeightParams& parm, DistanceFuncEnum f = DistanceFuncEnum::kSphericalManhattan) const final;
		std::vector<int> GetSquareTileIds(const int TileId1, const int TileId2) const final;
		std::vector<int> GetTriangleTileIds(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& center, const glm::vec3& vecDir = {}) const final;
		//------------------------------None Virtual func
		/*
		 * 初始化一个球，生成Tile和SpanList handle，并赋予MeshData
		 */
		SphereSegmentMgr(float radius, int tileSize, float cellSize);
		inline float GetRadius() const { return radius; }

	private:
		float radius = 0.0f;
		float unitRadianSize_ = 0.05f;
	private:
		//------------------------------Virtual func
		
		int GetSpanListIndexFromWorldPos(float x, float y, float z)const final;
		int OffsetGetEdgeSpanListNeighborIndex(const struct Cell& sl, EdgeNeighborDirect) final;
		std::tuple<int, int> Get2TileIndexFromWorldPos(float x, float y, float z) const final;

		//------------------------------None Virtual func
		int GetLatitudeIndex(float x, float y, float z) const;//get angle with y axis
		bool CheckConeection(int cellId1,int cellId2);
		/*
		 * 给予世界空间下的x,y,z点，获取Tile二维数组的索引(本球中)
		 */
		
	};
}

