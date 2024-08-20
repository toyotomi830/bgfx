#include "Segmentation.hh"
namespace segment
{
    Tile Segmentation::getTile(int i,int j) const{
        return tileList[i,j];
    }

    const Cell* Segmentation::getCell(int index) const{
        return cellList[index];
    }
} // namespace segment
