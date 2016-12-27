#ifndef CIRCLEGRAPHICSITEM_H
#define CIRCLEGRAPHICSITEM_H

#include <CGAL/Qt/Converter.h>
#include <CGAL/Qt/GraphicsItem.h>

namespace CGAL {
namespace Qt {

template <typename K>
class CircleGraphicsItem: public QGraphicsEllipseItem
{
	typedef typename K::Circle_2 Circle_2;

public:
	CircleGraphicsItem(Circle_2 *circle);

protected:
	CGAL::Qt::Converter<K> convert;
};

template <typename K>
CircleGraphicsItem<K>::CircleGraphicsItem(Circle_2 *circle)
{
	setRect(convert(circle->bbox()));
	setPen(QPen(::Qt::black, 0, ::Qt::SolidLine));
}

} // namespace Qt
} // namespace CGAL

#endif // CIRCLEGRAPHICSITEM_H
