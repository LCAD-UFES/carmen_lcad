#ifndef MVOG_GTK_GUI_MAP_DRAWER_2D_H
#define MVOG_GTK_GUI_MAP_DRAWER_2D_H

#include <GL/glut.h>

#include <mvog_model/map.h>
#include <mvog_gtk_gui/gtk_gui.h>

namespace MVOG
{

class GTKGui;

class MapDrawer2D
{
	private:

		Map    * map_;
    GTKGui * gui_;

    bool drawProbability;
    float visSize;

    void drawGrid();
    void drawProbabilities(float z);

	public:

		MapDrawer2D(GTKGui * gui);
		~MapDrawer2D();

    void draw();

		void setView();
		void setDrawProbability(bool p) { drawProbability = p; }
		void setVisSize(float z) { visSize = z; }
		//void getVisSize() { return visSize; }

		void setMap(MVOG::Map * map) { map_ = map; }
		Map * getMap() {return map_;}
};

} // namespace MVOG

#endif // MVOG_GTK_GUI_MAP_DRAWER_2D_H
