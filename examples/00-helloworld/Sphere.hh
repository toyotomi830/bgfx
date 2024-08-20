#ifndef SPHERE_HH
#define SPHERE_HH
#include "Segmentation.hh"
namespace segment
{
    class SphereSegmentation : Segmentation
    {
    private:
        float radius = 0.0f;
	    float unitRadianSize_ = 0.05f;
        float latitudeStride;
        int GetLatitudeIndex(float x, float y, float z) const;//get angle with y axis
        int GetLatitudeIndex(glm::vec3 pos) const;//get angle with y axis
        int OffsetGetEdgeSpanListNeighborIndex(const struct Cell& sl, NeighborDirection);
        void findNeighbor();
		/*list every tile matched with this position*/
		std::vector<std::tuple<int, int>> getOverLayTile(const glm::vec3& xyz);
		/*return cell index from given tile*/
		int getCellIndexFormTile(Tile, glm::vec3);
    public:
        SphereSegmentation(float radius, int tileSize, float cellSize);
		inline float GetRadius() const { return radius; }
        /*
            get tile index from world pos
        */
        int GetTileIndex(const glm::vec3& xyz, const glm::vec3& vecDir = {}) const;
        std::tuple<int,int> GetTileIndex2(const glm::vec3& xyz, const glm::vec3& vecDir = {}) const;
        float GetCellDistance(const CellSpanId start,const CellSpanId end, const HeightParams param);
		int GetCellIndex(const glm::vec3) const;
    };
    
} // namespace segment


#endif // SPHERE_HH
