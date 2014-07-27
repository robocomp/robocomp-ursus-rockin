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

#include "plantwidget.h"


PlantWidget::PlantWidget(QFrame *parent)
{
	setScene(new QGraphicsScene(this));
	QPixmap px("planta.png");
	px = px.scaled(size().width()-100,size().height()-20);
	this->scene()->addPixmap(px);
	setParent(parent);
	setMouseTracking(true);
	setFrameStyle(QFrame::Panel | QFrame::Sunken);
	
}

void PlantWidget::mouseMoveEvent(QMouseEvent *event)
{
	//qDebug() << event->pos();
	QList<QPair<QPointF,QPointF> > intervals;
	intervals.append(QPair<QPointF,QPointF>(QPointF(83,447),QPointF(0,9000))); 
	intervals.append(QPair<QPointF,QPointF>(QPointF(65,383),QPointF(0,-9000))); 
	QMat m = QMat::afinTransformFromIntervals( intervals );
	QVec p = m * QVec::vec3(event->x(), event->y(), 1);
	QVec res = QVec::vec3(p.x(), 0 , p.y());
	emit mouseMove(res);
	
}

void PlantWidget::mousePressEvent(QMouseEvent *event)
{
	//qDebug() << event->pos();
	QList<QPair<QPointF,QPointF> > intervals;
	intervals.append(QPair<QPointF,QPointF>(QPointF(83,447),QPointF(0,9000))); 
	intervals.append(QPair<QPointF,QPointF>(QPointF(65,383),QPointF(0,-9000))); 
	QMat m = QMat::afinTransformFromIntervals( intervals );
	QVec p = m * QVec::vec3(event->x(), event->y(), 1);
	QVec res = QVec::vec3(p.x(), 0 , p.y());
	emit mousePress(res);
	
}