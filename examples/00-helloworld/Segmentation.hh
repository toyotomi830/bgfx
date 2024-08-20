#ifndef SEGMENTATION_HH
#define SEGMENTATION_HH
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

namespace segment
{
    enum NeighborDirection{
        /*rotation around axis Y positive*/
        POSY=0,
        /*rotation around axis Y negtive*/
        NEGY=1,
         /*rotation around axis X positive*/
        POSX=2,
         /*rotation around axis X negtive*/
        NEGX=3
    };
    /*
        Basic unit in the space, contain Cartesian coordinate, span data, tiles, and available neighbors in 4 directions
    */
    struct Cell
    {
       glm::vec3 CenteralWorldPos;
       /*
            Cell's 4 direction neighbor, -1 as unavailable
       */
       std::array<int,4> neighborIndex;
       /*
            Listing all tiles match with this cell
       */
       std::vector<size_t> tileList;
       std::vector<Span> spans;

       CellId id;
    }; 
    /*
        frist = cellIndex
        second = spanIndex
    */
    typedef std::tuple<size_t,size_t> CellSpanId ;
    /*
        frist = cellptr
        second = spanIndex
    */
    typedef std::tuple<const Cell*,size_t> CellSpan ;
    
    /*
        update with server;
    */
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
        ??Consider and nrigbor relation in Tile
        ??consider put cells index in Tile
    */
    struct Tile
    {
        glm::vec3 centerPos = { 0, 0, 0 };

        glm::vec3 axis_u = { 0, 0, 0 };

		glm::vec3 axis_v = { 0, 0, 0 };

        int tileIndex = 0;
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

    class Segmentation
    {
    protected:
        int tileSize = 0;
        float cellSize = 0.0f;
        /*
            tile metrix by latitude X longitude
            each vector for same latitude may not have same size;
            v0: t0
            v1: t1 t2 t3
            v2: t4 t5 t6 t7 
            ext..
        */
		std::vector<std::vector<Tile>> tileList;
        int tileNums;
		/*
            storage for real cell data, size will not match with tiles, overlayed area will only store one unique cell data
        */
        std::vector<Cell> cellData;
    private:
        
    public:
        /*
            get tile index from world pos
        */
        virtual int GetTileIndex(const glm::vec3& xyz, const glm::vec3& vecDir = {}) const = 0;
		virtual int GetCellIndex(const glm::vec3) const = 0;
        virtual std::tuple<int,int> GetTileIndex2(const glm::vec3& xyz, const glm::vec3& vecDir = {})const = 0;
        virtual float GetCellDistance(const CellSpanId start,const CellSpanId end, const HeightParams param) =0;
        /* Get tile by index*/
        Tile getTile(int ,int) const;
        /*Get Cell ptr by index*/
        const Cell* getCell(int index) const;
        inline int getTileNums() const {return tileNums;}
		inline int GetTileSize() const { return tileSize; }
		inline float GetCellSize() const { return cellSize; }
        inline int CellIdGetIndice(const CellId& handle) const
		{
			return handle.tileId * tileSize * tileSize + handle.x * tileSize + handle.z;
		}
        /*sphere id*/
        int id;
        glm::vec3 center = {0,0,0};
		/*
			pointer to cellData, match with tiles, will contain same address due to overlayed area
			size = \sum_{i=0}^{tileList.size()}{\sum_{j=0}^{tileList[i].size())}{tileSize^{2}}}
		*/
		std::vector<Cell*> cellList;
    };
    
    
} // namespace segment
#endif // SEGMENTATION_HH
