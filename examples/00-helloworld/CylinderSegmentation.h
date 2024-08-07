#pragma once
#include"SegmentationBase.h"
namespace segment_mgr
{
	class CylinderSegmentMgr : public SegmentMgr
	{
	public:
		//---------------------------Non Virtual Func
		CylinderSegmentMgr(float _radius, float _height, int _tileSize, float _cellSize);
		//---------------------------Virtual Func
		int GetTileIndexFromWorldPos(const glm::vec3& xyz, const glm::vec3& center = {}, const glm::vec3& vecDir = {}) const final;
		const CellPos3D GetCellId3DFromWorldPos(const glm::vec3& xyz, const glm::vec3& center = {}, const HeightParams& parm = {},
			const glm::vec3& vecDir = {}) const final;
		std::tuple<float, float> GetSpanHeight(unsigned int smin, unsigned smax, const HeightParams& parm) const final;
		glm::vec3 GetWorldPosFromCellId3D(const CellPos3D& Id, const glm::vec3& Center, const HeightParams& Parm, const glm::vec3& vecDir = {}) const final;
		float GetCellId3dDistance(const CellPos3D& s1, const CellPos3D& s2, const HeightParams& parm, DistanceFuncEnum f = DistanceFuncEnum::kSphericalManhattan) const final;
		std::vector<int> GetSquareTileIds(const int TileId1, const int TileId2) const final;
		std::vector<int> GetTriangleTileIds(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& center,
			const glm::vec3& vecDir = {}) const final;
		const float GetRadius() const { return radius; }
		glm::mat4 ToWorldPosMat(const glm::vec3& vecDir) const;
		std::tuple<glm::vec3, glm::vec3> GetTileWorldAABB(const Tile& t, float top, const glm::vec3& vecDir) const;

	private:
		float radius;
		float height;
	private:
		//------------------------------Virtual func
		int GetSpanListIndexFromWorldPos(float x, float y, float z) const final;
		int OffsetGetEdgeSpanListNeighborIndex(const struct Cell& sl, EdgeNeighborDirect) final;
		std::tuple<int, int> Get2TileIndexFromWorldPos(float x, float y, float z) const final;
	};
}

