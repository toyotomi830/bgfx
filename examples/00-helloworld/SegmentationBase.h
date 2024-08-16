#pragma once

#include <functional>
#include <algorithm>
#include <vector>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include <memory>
#include <array>
#include "SpanData.h"

#define M_PI 3.1415926535


namespace segment_mgr
{
	struct CellId
	{
		union
		{
			struct
			{
				uint16_t tileId;
				uint8_t x;
				uint8_t z;
			};
			uint32_t id = 0;
		};
		CellId() = default;
		CellId(uint32_t _id) : id(_id) {}
		CellId(uint16_t tI, uint8_t xI, uint8_t zI) : tileId(tI), x(xI), z(zI) {}
	};

	/*
	* 和CellId相比，多了一个y轴分量，理应是voxelCellHeight的整数倍
	*/
	struct CellPos3D : public CellId
	{
		uint16_t y = 0;
		CellPos3D() = default;
		CellPos3D(CellId id, uint16_t _y) : CellId(id), y(_y) {}
		CellPos3D(uint16_t _tileId, uint8_t _x, uint16_t _y, uint8_t _z) : CellId(_tileId, _x, _z), y(_y) {}
	};

	/*
	 * 对Cell找寻临近点用,四个方向
	 */
	enum EdgeNeighborDirect { kNegX = 1, kX = 2, kNegZ = 3, kZ = 4 };

	/*
	 * 三种计算两个点距离的方式
	 * Euclidean：3维空间中的之间距离
	 * Spherical：在球面上的距离，等同于弧长
	 * SphericalManhattan：球面曼哈顿距离
	 */
	enum DistanceFuncEnum { kEuclidean = 1, kSpherical = 2, kSphericalManhattan = 3 };

	/*
	 * 保存有SpanList的数据，主要用来告诉我们邻居的索引
	 */

	struct Cell
	{
		friend class SegmentMgr;
		friend class SphereSegmentMgr;
		friend class PlaneSegmentMgr;
		friend class CylinderSegmentMgr;

		// SpanList的底点在以{0，0，0}为球心的3D空间下的位置，后续可能会删掉；

		glm::vec3 centerPos;
		Cell(const glm::vec3& Pos) : centerPos(Pos) {}
		SpanList spanlist;

	//private:
		// 邻居的索引；
		std::array<int, 4> neighborsIndex;
	};

	class Tile
	{
	public:
		glm::vec3 centerPos = { 0, 0, 0 };  // Tile 的中心位置

		glm::vec3 axis_u = { 0, 0, 0 };

		glm::vec3 axis_v = { 0, 0, 0 };
		// 两个基向量

		int tileIndex = 0;  // 在本球中，这个Tile是第几个
	};

	struct HeightParams
	{
		/*
		* 相较于半径的底面，可能为负
		*/
		float bottom = 0.0f;
		float cellHeight = 0.0f;

		HeightParams() = default;
		HeightParams(float _b, float _c) :bottom(_b), cellHeight(_c)
		{

		}

		template<class T>
		HeightParams(const T& data)
		{
			bottom = data.bottom;
			cellHeight = data.cellHeight;
		}
	};
	class SegmentMgr
	{
	protected:
		int tileSize = 0;
		float cellSize = 0.0f;
		int totalTilesNum = 0;
		std::vector<std::vector<Tile>> tiles;
	public:
		std::vector<Cell> listData;
		std::vector<CellId> handleData;
	public:
		//------------------------------pure virtual func
		virtual int GetTileIndexFromWorldPos(const glm::vec3& xyz, const glm::vec3& center = {}, const glm::vec3& vecDir = {}) const = 0;
		virtual const CellPos3D GetCellId3DFromWorldPos(const glm::vec3& xyz, const glm::vec3& center = {}, const HeightParams& parm = {}, const glm::vec3& vecDir = {}) const = 0;
		virtual std::tuple<float, float> GetSpanHeight(unsigned int smin, unsigned smax, const HeightParams& parm) const = 0;
		virtual glm::vec3 GetWorldPosFromCellId3D(const CellPos3D& Id, const glm::vec3& Center, const HeightParams& Parm, const glm::vec3& vecDir = {}) const = 0;
		virtual float GetCellId3dDistance(const CellPos3D& s1, const CellPos3D& s2, const HeightParams& parm, DistanceFuncEnum f = DistanceFuncEnum::kSphericalManhattan) const = 0;
		virtual std::vector<int> GetSquareTileIds(const int TileId1, const int TileId2) const = 0;
		virtual std::vector<int> GetTriangleTileIds(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& center, const glm::vec3& vecDir = {}) const = 0;
		//------------------------------Virtual func
		virtual const Tile& GetTileByIndex(int index) const;

		//------------------------------None Virtual func
		std::vector<CellId> GetCellIdNeighbors(CellId& sph) const;
		std::tuple<glm::vec3, glm::vec3> GetTileWorldAABB(const Tile& t, float top) const;
		std::tuple<glm::vec3, glm::vec3> GetTileSectorWorldAABB(const Tile& t, float near, float far) const;
		std::tuple<glm::vec3, glm::vec3> GetCell3DAabb(const CellPos3D& c3d, const HeightParams& parm) const;
		inline int GetTotalTileNums() const { return totalTilesNum; }
		inline int GetTileSize() const { return tileSize; }
		inline float GetCellSize() const { return cellSize; }
		inline int CellIdGetIndice(const CellId& handle) const
		{
			return handle.tileId * tileSize * tileSize + handle.x * tileSize + handle.z;
		}
	protected:
		//------------------------------pure virtual func
		virtual int GetSpanListIndexFromWorldPos(float x, float y, float z)const = 0;
		virtual int OffsetGetEdgeSpanListNeighborIndex(const struct Cell& sl, EdgeNeighborDirect) = 0;
		virtual std::tuple<int, int> Get2TileIndexFromWorldPos(float x, float y, float z) const = 0;
		//------------------------------Virtual func

		//------------------------------None Virtual func
		std::tuple<int, int> Get2TileIndexFrom1TileIndex(int Index) const;

		

	};
}
