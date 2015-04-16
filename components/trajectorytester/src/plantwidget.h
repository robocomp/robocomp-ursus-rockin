/*
 * Copyright 2014 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef PLANTWIDGET_H
#define PLANTWIDGET_H

#include <QObject>
#include <QMouseEvent>
#include <QGraphicsView>
#include <qmat/QMatAll>
 #include <QGraphicsPixmapItem>

class PlantWidget : public QGraphicsView
 {
	Q_OBJECT

	public:
		PlantWidget(QFrame* parent, QPointF pxCorner_, QPointF RXCorner_, QPointF pzCorner_, QPointF PZCorner_);
		void moveRobot(float x, float y, float alfa);

	private:
		void mouseMoveEvent(QMouseEvent *event);
		void mousePressEvent(QMouseEvent *event);
		QPointF pxCorner, pzCorner, RXCorner, RZCorner;
		QGraphicsPixmapItem *robPtr;
		QList<QPair<QPointF,QPointF> > intervals, intervalsI;
		QMat m, mI;
		
	signals:
		void mouseMove(QVec);
		void mousePress(QVec);
	 
};

#endif // PLANTWIDGET_H
