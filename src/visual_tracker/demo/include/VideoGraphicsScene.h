/*
 * Copyright © 2011 Paul Nader 
 *
 * This file is part of QOpenTLD.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VIDEOGRAPHICSSCENE_H_
#define VIDEOGRAPHICSSCENE_H_

#include <QMouseEvent>
#include <QGraphicsScene>

class SelectionRectange;
class QGraphicsSceneMouseEvent;

class VideoGraphicsScene : public QGraphicsScene
{
	Q_OBJECT

public:
	VideoGraphicsScene(QObject *parent=0);
	~VideoGraphicsScene() {}

	void setSelection(QRectF& rect);
	QRectF getSelection(void) const;
	void showSelection(bool show = true) const;
	bool selectionVisible(void) const;

signals:
	void selectionChanged(const QRectF rect);
	void status(const QString message);
	void startSelection();
	void stopSelection();
	void regionSelected();

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent *e);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *e);
	void mouseMoveEvent(QGraphicsSceneMouseEvent* e);
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* e);

private:
	bool m_selecting;
	QPointF m_selectionOrigin, m_selectionFinal;
	SelectionRectange *m_selectionRectangle;
};



#endif /* VIDEOGRAPHICSSCENE_H_ */
